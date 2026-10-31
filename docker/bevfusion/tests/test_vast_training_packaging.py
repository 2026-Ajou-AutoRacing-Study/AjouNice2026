import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/build_vast_training_image.py'
spec = importlib.util.spec_from_file_location('vast_packaging', SCRIPT)
packaging = importlib.util.module_from_spec(spec)
spec.loader.exec_module(packaging)


class TrainingPackagingTests(unittest.TestCase):
    def test_context_excludes_payloads_and_credentials(self):
        paths = [str(relative) for _, relative in packaging.selected()]
        self.assertTrue(paths)
        self.assertTrue(all(Path(name).suffix not in ('.pth', '.pt', '.pkl', '.png', '.jpg', '.bag') for name in paths))
        self.assertTrue(all(not any(part in ('.git', '.aws', '.codex', '.ssh', 'artifacts', 'wandb')
                                   for part in Path(name).parts) for name in paths))
        self.assertFalse(any(Path(name).name in ('.env', '.netrc', 'config.json') for name in paths))

    def test_training_config_and_runner_present(self):
        paths = {str(relative) for _, relative in packaging.selected()}
        self.assertIn('perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py', paths)
        self.assertIn('perception/detection3d/bevfusion/tools/run_base5_direct_training.py', paths)
        self.assertIn('perception/detection3d/common/evaluation/morai_detection.py', paths)

    def test_existing_context_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'never overwritten'):
                packaging.prepare(Path(directory))

    def test_no_automatic_training_or_host_secrets(self):
        dockerfile = (SCRIPT.parents[1] / 'Dockerfile.vast-training').read_text()
        self.assertNotIn('COPY . ', dockerfile)
        self.assertNotIn('WANDB_API_KEY', dockerfile)
        self.assertNotIn('docker run', dockerfile)
        self.assertIn('CMD ["bash"]', dockerfile)


if __name__ == '__main__':
    unittest.main()
