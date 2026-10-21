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
        cfg.val_evaluator=Attr(type='MoraiBase5MergedCenterMetric')
        state = dict(cohorts=[dict(name=n, root='/dataset/'+n,
            split_counts=dict(train=1, val=0 if n == 'foggy13_completion' else 1))
            for n in m.EXPECTED],val_token_to_cohort={'native_token':'sunny11'})
        m.attach_concat(cfg, state)
        self.assertEqual(state['train_pipeline'], pipe)
        for split in ('train', 'val'):
            loader = cfg[split+'_dataloader']
            wrapper = loader.dataset
            self.assertEqual(wrapper['ignore_keys'], ['layout_split_overlay_sha256'])
            active = [c for c in state['cohorts'] if c['split_counts'][split] > 0]
            self.assertEqual(len(wrapper['datasets']), 10 if split == 'train' else 9)
            self.assertEqual(loader.sampler.shuffle, split == 'train')
            for child, cohort in zip(wrapper['datasets'], active):
                self.assertEqual(child.data_root, cohort['root'])
                self.assertEqual(child.pipeline, pipe)
                self.assertFalse(child.filter_empty_gt)
                self.assertTrue(child.ann_file.endswith('_'+split+'.pkl'))
        self.assertEqual(cfg.val_evaluator['base5_val_logging']['token_to_cohort'],
                         {'native_token':'sunny11'})
        self.assertNotIn('foggy13_completion',
                         cfg.val_evaluator['base5_val_logging']['expected_cohort_frames'])

    def test_completion_restores_total_without_changing_update_count(self):
        self.assertEqual(sum(m.EXPECTED.values()), 30750)
        self.assertEqual(m.EXPECTED['foggy13_completion'], 1)
        self.assertEqual(m.EXPECTED_COUNTS, dict(train=26136, val=4614, test=0))
        self.assertEqual((m.EXPECTED_COUNTS['train'] + 3) // 4, 6534)

    def test_validation_extension_is_pinned_to_the_reviewed_file(self):
        self.assertEqual(m.VAL_LOGGING_SCHEMA, 'base5_val_logging_v1')
        self.assertEqual(m.sha(HERE/'base5_val_logging.py'), m.VAL_LOGGING_SHA)

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
