"""CPU admission tests for the optional bounded target-removal benchmark."""
import copy
import unittest

from scripts.runtime import benchmark_base5_training_extension as wrapper


def proof():
    row = dict(targets_exact=True, losses_exact=True, finite=True, RNG_exact=True,
               gradient=dict(exact=True, max_abs=0.), baseline_losses={'loss': 1.},
               candidate_losses={'loss': 1.})
    return dict(complete=True, valid=True, expected_fixture_batches=3,
        contract_sha256='contract', checkpoint_sha256='checkpoint',
        extension=dict(version=wrapper.VERSION, script_sha256='extension'),
        compile_camera=False, no_optimizer_step=True, no_checkpoint_write=True,
        model_original_unmodified=True, fixtures=[copy.deepcopy(row) for _ in range(3)])


class GateTests(unittest.TestCase):
    def test_exact_complete_gate_passes(self):
        wrapper.validate_qualification(proof(), 'extension', 'checkpoint', 'contract')

    def test_partial_or_wrong_snapshot_rejected(self):
        row = proof(); row['complete'] = False
        with self.assertRaises(ValueError):
            wrapper.validate_qualification(row, 'extension', 'checkpoint', 'contract')
        with self.assertRaises(ValueError):
            wrapper.validate_qualification(proof(), 'extension', 'another_checkpoint', 'contract')

    def test_one_nonexact_gradient_rejected(self):
        row = proof(); row['fixtures'][1]['gradient']['max_abs'] = 1e-8
        with self.assertRaises(ValueError):
            wrapper.validate_qualification(row, 'extension', 'checkpoint', 'contract')

    def test_model_or_loss_change_rejected(self):
        for field in ('targets_exact', 'losses_exact', 'RNG_exact', 'finite'):
            row = proof(); row['fixtures'][0][field] = False
            with self.assertRaises(ValueError):
                wrapper.validate_qualification(row, 'extension', 'checkpoint', 'contract')


if __name__ == '__main__': unittest.main()
