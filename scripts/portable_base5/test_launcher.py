import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).with_name('start_morai_capture.sh')
SOURCE = SCRIPT.read_text().split("<<'PY'\n", 1)[1].rsplit('\nPY', 1)[0]
NAMESPACE = {'__name__': 'portable_morai_launcher_test'}
exec(compile(SOURCE, str(SCRIPT), 'exec'), NAMESPACE)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.workspace = Path(self.temp.name) / 'ajounice_base5_capture'
        self.bundle = self.workspace / 'capture_bundle'
        self.mr = Path(self.temp.name) / 'MoraiLauncher_Lin'
        self.original = self.bundle / 'morai/MoraiLauncher_Lin'
        for root in (self.mr, self.original):
            for relative in NAMESPACE['CORE']:
                file = root / relative
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_text('#!/bin/sh\nexit 93\n')
                file.chmod(0o755 if relative.endswith('.x86_64') else 0o644)
        bwrap = self.bundle / 'bin/bwrap'
        bwrap.parent.mkdir(parents=True)
        bwrap.write_text('#!/bin/sh\nexit 94\n')
        bwrap.chmod(0o755)
        for base in (self.workspace, self.mr):
            for directory in (base, *[p for p in base.rglob('*') if p.is_dir()]):
                directory.chmod(0o755)
        (self.workspace / 'work').mkdir(exist_ok=True)
        (self.workspace / 'work').chmod(0o755)

    def plan(self, environment=None):
        return NAMESPACE['make_plan'](self.workspace, {} if environment is None else environment)

    def test_readonly_plan_does_not_create_session(self):
        before = sorted(p.relative_to(self.workspace).as_posix() for p in self.workspace.rglob('*'))
        plan = self.plan()
        self.assertFalse(plan['gui_ready'])
        self.assertFalse(Path(plan['session']).exists())
        self.assertEqual(before, sorted(p.relative_to(self.workspace).as_posix() for p in self.workspace.rglob('*')))
        self.assertFalse(plan['morai_started'])

    def test_cli_default_is_readonly_and_json(self):
        result = subprocess.run(['bash', str(SCRIPT), '--workspace', str(self.workspace)],
                                capture_output=True, text=True, env={'PATH': '/usr/bin:/bin'}, check=True)
        self.assertEqual(json.loads(result.stdout)['working_morai'], str(self.mr))
        self.assertFalse((self.workspace / 'work/morai-session').exists())

    def test_run_without_display_never_creates_files(self):
        result = subprocess.run(['bash', str(SCRIPT), '--workspace', str(self.workspace), '--run'],
                                capture_output=True, text=True, env={'PATH': '/usr/bin:/bin'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('graphical desktop', result.stderr)
        self.assertFalse((self.workspace / 'work/morai-session').exists())

    def test_sandbox_scope_gpu_and_no_sudo(self):
        plan = self.plan({'DISPLAY': ':0', 'XDG_RUNTIME_DIR': '/run/user/1000',
                          'LD_PRELOAD': '/unsafe.so', 'SSH_AUTH_SOCK': '/unsafe.sock'})
        command = plan['command']
        self.assertEqual(command[command.index('--ro-bind') + 1:command.index('--ro-bind') + 3], ['/', '/'])
        self.assertIn('--dev-bind', command)
        self.assertIn('--tmpfs', command)
        self.assertEqual(command[command.index('--bind') + 1:command.index('--bind') + 3],
                         [str(self.workspace), str(self.workspace)])
        self.assertEqual(plan['environment']['DISPLAY'], ':0')
        self.assertEqual(plan['environment']['XDG_RUNTIME_DIR'], '/run/user/1000')
        for key in ('LD_PRELOAD', 'SSH_AUTH_SOCK'):
            self.assertNotIn(key, plan['environment'])
        self.assertNotIn('sudo', ' '.join(command))
        self.assertNotIn('MORAISim.sh', ' '.join(command))
        self.assertEqual(plan['environment']['HOME'], str(self.workspace.parent))
        writable = [command[i + 1:i + 3] for i, value in enumerate(command) if value == '--bind']
        self.assertEqual(writable, [[str(self.workspace), str(self.workspace)], [str(self.mr), str(self.mr)]])
        self.assertIn('--ro-bind', command)
        for key in ('XDG_CONFIG_HOME', 'XDG_CACHE_HOME', 'TMPDIR', 'CUDA_CACHE_PATH', '__GL_SHADER_DISK_CACHE_PATH'):
            self.assertTrue(plan['environment'][key].startswith(str(self.workspace) + '/'))

    def test_run_execs_only_fake_bwrap_and_never_falls_back(self):
        plan = self.plan()
        plan['gui_ready'] = True
        with mock.patch.object(os, 'execve', side_effect=OSError('fake sandbox failure')) as execute:
            with self.assertRaisesRegex(OSError, 'fake sandbox failure'):
                NAMESPACE['run_plan'](plan)
        execute.assert_called_once_with(str(self.bundle / 'bin/bwrap'), plan['command'], plan['environment'])
        self.assertTrue(Path(plan['session']).is_dir())

    def test_preserves_explicit_xauthority(self):
        authority = Path(self.temp.name) / 'desktop-authority'
        authority.write_text('fixture')
        plan = self.plan({'XAUTHORITY': str(authority)})
        self.assertEqual(plan['environment']['XAUTHORITY'], str(authority))

    def test_original_home_authority_is_not_replaced_by_session_home(self):
        home = Path(self.temp.name) / 'desktop-home'
        home.mkdir()
        (home / '.Xauthority').write_text('fixture')
        self.assertEqual(self.plan({'HOME': str(home)})['environment']['XAUTHORITY'], str(home / '.Xauthority'))

    def test_rejects_symlinked_workspace(self):
        alias = Path(self.temp.name) / 'alias'
        alias.symlink_to(self.workspace, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            NAMESPACE['make_plan'](alias, {})

    def test_rejects_wrong_root_basename(self):
        wrong = Path(self.temp.name) / 'wrong'
        wrong.mkdir()
        wrong.chmod(0o755)
        with self.assertRaisesRegex(ValueError, 'basename'):
            NAMESPACE['make_plan'](wrong, {})

    def test_rejects_changed_or_hardlinked_core(self):
        executable = self.mr / 'MoraiLauncher_Lin.x86_64'
        executable.write_text('changed')
        with self.assertRaisesRegex(ValueError, 'differs'):
            self.plan()
        executable.unlink()
        os.link(self.original / executable.name, executable)
        with self.assertRaisesRegex(ValueError, 'hardlinked'):
            self.plan()

    def test_rejects_session_symlink_and_writable_workspace(self):
        (self.workspace / 'work/morai-session').symlink_to(self.temp.name)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            self.plan()
        (self.workspace / 'work/morai-session').unlink()
        self.workspace.chmod(0o777)
        with self.assertRaisesRegex(ValueError, 'writable'):
            self.plan()

    def test_discovery_does_not_depend_on_current_directory(self):
        script = self.bundle / 'tools/start_morai_capture.sh'
        self.assertEqual(NAMESPACE['discover_workspace'](script), self.workspace)

    def test_permission_helper_is_cwd_independent_and_stays_in_short_morai(self):
        helper = self.mr / 'setPermission.sh'
        helper.write_bytes(SCRIPT.with_name('morai_set_permission.sh').read_bytes())
        outside = Path(self.temp.name) / 'unrelated'
        outside.write_text('preserve')
        outside.chmod(0o640)
        before = outside.stat().st_mode
        result = subprocess.run(['bash', str(helper)], cwd=self.temp.name,
                                env={'HOME': self.temp.name, 'PATH': '/usr/bin:/bin'}, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(outside.stat().st_mode, before)
        self.assertEqual(outside.read_text(), 'preserve')
        self.assertEqual((self.mr / NAMESPACE['CORE'][-1]).stat().st_mode & 0o111, 0o111)

    def test_permission_helper_refuses_wrong_home_without_changes(self):
        helper = self.mr / 'setPermission.sh'
        helper.write_bytes(SCRIPT.with_name('morai_set_permission.sh').read_bytes())
        before = {p: p.stat().st_mode for p in self.mr.rglob('*')}
        result = subprocess.run(['bash', str(helper)], env={'HOME': '/nonexistent', 'PATH': '/usr/bin:/bin'},
                                capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(before, {p: p.stat().st_mode for p in self.mr.rglob('*')})


if __name__ == '__main__':
    unittest.main()
