import ast
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('full_data_prepare', HERE/'full_data_prepare.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
REPO = HERE.parents[1]


class Attr(dict):
    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError:
            raise AttributeError(key)

    __setattr__ = dict.__setitem__


class Tests(unittest.TestCase):
    def test_exact_original_five_site_adapter_compiles(self):
        core = REPO/m.CORE_REL
        self.assertEqual(m.sha(core), m.CORE_SHA)
        fn = m.adapted_prepare(core.read_text(), {})
        self.assertTrue(callable(fn))

    def test_modified_count_statement_rejected(self):
        text = (REPO/m.CORE_REL).read_text().replace(
            "len(pickle.loads(train_info.read_bytes())['data_list'])", '999')
        with self.assertRaisesRegex(ValueError, 'train-count'):
            m.adapted_prepare(text, {})

    def test_missing_attach_site_rejected(self):
        text = (REPO/m.CORE_REL).read_text().replace(
            'cfg.test_dataloader = None', 'cfg.unrelated = None')
        with self.assertRaisesRegex(ValueError, 'five'):
            m.adapted_prepare(text, {})

    def test_concat_preserves_children_pipeline_roots_sampler(self):
        pipe = [dict(type='LoadMultiViewImageFromFiles'), dict(type='Pack3DDetInputs')]
        cfg = Attr({s+'_dataloader': Attr(dataset=Attr(type='MoraiDataset',
            pipeline=copy.deepcopy(pipe), data_root='/old', filter_empty_gt=False),
            sampler=Attr(type='DefaultSampler', shuffle=s == 'train'))
            for s in ('train', 'val')})
        state = dict(cohorts=[dict(name=n, root='/dataset/'+n) for n in m.EXPECTED])
        m.attach_concat(cfg, state)
        self.assertEqual(state['train_pipeline'], pipe)
        for split in ('train', 'val'):
            loader = cfg[split+'_dataloader']
            wrapper = loader.dataset
            self.assertEqual(wrapper['ignore_keys'], ['layout_split_overlay_sha256'])
            self.assertEqual(len(wrapper['datasets']), 9)
            self.assertEqual(loader.sampler.shuffle, split == 'train')
            for child, cohort in zip(wrapper['datasets'], state['cohorts']):
                self.assertEqual(child.data_root, cohort['root'])
                self.assertEqual(child.pipeline, pipe)
                self.assertFalse(child.filter_empty_gt)
                self.assertTrue(child.ann_file.endswith('_'+split+'.pkl'))

    def test_incomplete_index_rejected_before_validator(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'index.json'
            path.write_text(json.dumps(dict(schema='base5_full_cohort_index_v1', complete=False)))
            with self.assertRaisesRegex(ValueError, 'not complete'):
                m.inspect_index(path, lambda *a, **k: self.fail('validator must not run'))

    def test_wrong_scope_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'index.json'
            path.write_text(json.dumps(dict(
                schema='base5_full_cohort_index_v1', complete=True, cohorts=[])))
            with self.assertRaisesRegex(ValueError, 'Exact latest'):
                m.inspect_index(path, lambda *a, **k: None)

    def test_run_missing_authority_rejected_before_model_import(self):
        with self.assertRaisesRegex(ValueError, 'training-start authority'):
            m.main(['--source', '/does/not/exist', '--data-index', '/absent',
                '--work-dir', '/absent', '--init-checkpoint', '/absent', '--run'])

    def test_model_optimizer_schedule_function_ast_not_modified(self):
        tree = ast.parse((REPO/m.CORE_REL).read_text())
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare')
        original = ast.dump(fn)
        m.adapted_prepare((REPO/m.CORE_REL).read_text(), {})
        self.assertEqual(original, ast.dump(fn))


if __name__ == '__main__':
    unittest.main()
