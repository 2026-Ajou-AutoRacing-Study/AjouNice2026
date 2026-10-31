import random
import socket
import unittest

import numpy as np
import torch

from scripts.runtime.profile_base5_input_loader import SeededSubset, TimedCallable, payload_digest, select_indices


class FakeChild:
    def __init__(self, length): self.length = length
    def __len__(self): return self.length


class FakeDataset:
    def __init__(self, lengths): self.datasets = [FakeChild(n) for n in lengths]
    def __len__(self): return sum(map(len, self.datasets))
    def __getitem__(self, index):
        return dict(index=index, rgb=np.full((2, 3, 3), np.random.randint(0, 256), dtype=np.uint8),
                    box=torch.rand(7), py=random.random(), labels=np.array([0, 2]))


class InputLoaderTests(unittest.TestCase):
    def test_child_coverage_unique_and_stable(self):
        dataset = FakeDataset([4250] * 6 + [212] * 3 + [1])
        indices = select_indices(dataset, 64)
        self.assertEqual(indices, select_indices(dataset, 64))
        self.assertEqual(len(indices), 64)
        self.assertEqual(indices, sorted(set(indices)))
        offset = 0
        for child in dataset.datasets:
            self.assertTrue(any(offset <= i < offset + len(child) for i in indices))
            offset += len(child)

    def test_validation_child_coverage(self):
        dataset = FakeDataset([750] * 6 + [38] * 3)
        self.assertEqual(len(select_indices(dataset, 96)), 96)

    def test_reject_unbounded_and_insufficient(self):
        dataset = FakeDataset([20] * 10)
        for n in [0, 9, 97]:
            with self.assertRaises(ValueError): select_indices(dataset, n)

    def test_rng_independent_order_exact_pixels_gt(self):
        dataset = FakeDataset([100])
        forward = SeededSubset(dataset, [2, 7, 19], 261015)
        reverse = SeededSubset(dataset, [19, 7, 2], 261015)
        self.assertEqual([payload_digest(forward[i]) for i in range(3)],
                         [payload_digest(reverse[i]) for i in reversed(range(3))])

    def test_rng_state_unchanged(self):
        random.seed(4); np.random.seed(4); torch.manual_seed(4)
        py, npstate, cpu = random.getstate(), np.random.get_state(), torch.get_rng_state()
        SeededSubset(FakeDataset([20]), [5], 34)[0]
        self.assertEqual(py, random.getstate())
        self.assertTrue(np.array_equal(npstate[1], np.random.get_state()[1]))
        self.assertTrue(torch.equal(cpu, torch.get_rng_state()))

    def test_payload_hash_observes_gt_pixels_dtype(self):
        one = dict(rgb=np.array([1], dtype=np.uint8), box=torch.tensor([1.0]), label=2)
        two = dict(one, label=1)
        self.assertNotEqual(payload_digest(one), payload_digest(two))
        self.assertNotEqual(payload_digest(one), payload_digest(dict(one, rgb=np.array([2], dtype=np.uint8))))
        self.assertNotEqual(payload_digest(one), payload_digest(dict(one, rgb=np.array([1], dtype=np.float32))))

    def test_timer_passes_same_objects_and_return(self):
        item = dict(value=np.array([2]))
        records = {}
        def transform(value):
            self.assertIs(value, item)
            value['done'] = True
            return value
        wrapped = TimedCallable(transform, records, 'transform')
        self.assertIs(wrapped(item), item)
        self.assertTrue(item['done'])
        self.assertEqual(len(records['transform']), 1)

    def test_real_two_worker_loader_matches_single_process(self):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except PermissionError:
            self.skipTest('Sandbox does not permit PyTorch FD-sharing UNIX socket')
        indices = [2, 5, 8, 11, 14, 17, 20, 23]
        hashes = []
        for workers in [0, 2]:
            loader = torch.utils.data.DataLoader(
                SeededSubset(FakeDataset([30]), indices, 261015),
                batch_size=4, num_workers=workers, shuffle=False,
                **(dict(timeout=5) if workers else {}))
            hashes.append([payload_digest(batch) for batch in loader])
        self.assertEqual(hashes[0], hashes[1])


if __name__ == '__main__': unittest.main()
