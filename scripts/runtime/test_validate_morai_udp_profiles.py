import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('udp_profiles', Path(__file__).with_name('validate_morai_udp_profiles.py'))
profiles = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profiles)


class ProductionInventoryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for folder, names in profiles.PRODUCTION_PROFILE_FILES.items():
            directory = self.root / 'configs' / folder
            directory.mkdir(parents=True)
            for name in names:
                (directory / name).write_text('{}')

    def test_canonical_only(self):
        profiles.validate_production_inventory(self.root)

    def test_missing_canonical(self):
        (self.root / 'configs/sensors/Sensor_competition_local.json').unlink()
        with self.assertRaisesRegex(ValueError, 'missing='):
            profiles.validate_production_inventory(self.root)

    def test_explicit_team_development_exceptions_are_allowed(self):
        for folder,names in profiles.DEVELOPMENT_PROFILE_FILES.items():
            for name in names:
                (self.root/'configs'/folder/name).write_text('{}')
        profiles.validate_production_inventory(self.root)

    def test_duplicate_sensor_rejected(self):
        (self.root / 'configs/sensors/Sensor.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'unexpected=.*Sensor.json'):
            profiles.validate_production_inventory(self.root)

    def test_old_network_rejected(self):
        (self.root / 'configs/network/ego_network_1.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'unexpected='):
            profiles.validate_production_inventory(self.root)

    def test_legacy_does_not_count_as_current(self):
        legacy = self.root / 'configs/sensors/legacy'
        legacy.mkdir()
        (legacy / 'Sensor_4.json').write_text('{}')
        profiles.validate_production_inventory(self.root)

    def test_canonical_symlink_rejected(self):
        target = self.root / 'sensor-original.json'
        target.write_text('{}')
        canonical = self.root / 'configs/sensors/Sensor_competition_local.json'
        canonical.unlink()
        canonical.symlink_to(target)
        with self.assertRaisesRegex(ValueError, 'regular files'):
            profiles.validate_production_inventory(self.root)


if __name__ == '__main__':
    unittest.main()
