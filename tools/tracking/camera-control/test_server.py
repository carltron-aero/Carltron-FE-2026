"""Validation tests with no camera connection or hardware commands."""
import threading
import unittest
import queue
import copy
import tempfile
from pathlib import Path
from unittest.mock import patch, Mock
from server import App, Controller
from preview import dshow_name


class FakeController:
    def __init__(self):
        self.calls = []

    def call(self, data):
        self.calls.append(data)
        return {'readback': 'mock'}


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.app = App.__new__(App)
        self.app.transaction = threading.RLock()
        self.app.controller = FakeController()
        self.app.log = lambda *_: None
        self.app.state = {'Device': {'Path': 'camera-A'}}
        self.app.get_state = lambda: None

    def rejected(self, data):
        with self.assertRaises(ValueError):
            self.app.action(data)
        self.assertEqual(self.app.controller.calls, [])

    def test_requires_absolute_confirmation(self):
        for confirmed in [None, False, 1, 'true']:
            self.rejected({'action': 'absolute', 'pan': 0, 'tilt': 0, 'confirmed': confirmed})

    def test_rejects_non_integer_and_overflow(self):
        for value in [True, 1.0, '0', None, 2147483648, -2147483649]:
            self.rejected({'action': 'absolute', 'pan': value, 'tilt': 0, 'confirmed': True})

    def test_no_raw_vendor_endpoint(self):
        self.rejected({'action': 'vendor', 'selector': 2, 'value': 1})

    def test_confirmed_typed_request(self):
        data = {'action': 'absolute', 'pan': 43200, 'tilt': -25200, 'confirmed': True, 'device_path': 'camera-A'}
        self.assertEqual(self.app.action(data), {'readback': 'mock'})
        self.assertEqual(self.app.controller.calls, [data])


class ReconnectingController:
    def __init__(self):
        self.calls = []
        self.devices = []
        self.dead = False
        self.stale = False
        self.selected = None
        self.restarts = 0
        self.fail_write = None

    def restart(self):
        self.restarts += 1
        self.dead = self.stale = False
        self.selected = None

    def call(self, data):
        self.calls.append(data.copy())
        if self.dead:
            raise ConnectionError('Bridge stopped')
        if data['action'] == 'devices':
            return [d.copy() for d in self.devices]
        if data['action'] == 'select':
            self.selected = next(d for d in self.devices if d['Index'] == data['index'])
        if data['action'] in {'select', 'state'}:
            return {'Device': self.selected.copy(), 'Controls': [{'Supported': not self.stale}]}
        if data['action'] == self.fail_write:
            self.dead = True
            raise ConnectionError('Write timed out')
        return {'readback': 'mock'}


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.bridge = ReconnectingController()
        self.device = {'Index': 0, 'Name': 'PTZ Pro 2', 'Path': 'camera-A'}
        self.bridge.devices = [self.device]
        self.controller_patch = patch('server.Controller', return_value=self.bridge)
        self.snapshot_patch = patch.object(App, 'save_snapshot', return_value='mock-snapshot')
        self.log_patch = patch.object(App, 'log')
        for item in [self.controller_patch, self.snapshot_patch, self.log_patch]:
            item.start()
            self.addCleanup(item.stop)
        self.app = App(8765)
        self.bridge.calls.clear()

    def test_unplug_replug_restores_controls_without_writes(self):
        original_revision = self.app.connection_revision
        self.bridge.devices = []
        result = self.app.get_state()
        self.assertIsNone(result['state'])
        self.assertEqual(result['devices'], [])
        self.assertEqual(result['preferred_path'], 'camera-A')
        self.bridge.devices = [self.device]
        result = self.app.get_state()
        self.assertTrue(result['state']['Controls'][0]['Supported'])
        self.assertGreater(result['connection_revision'], original_revision)
        self.assertEqual([c['action'] for c in self.bridge.calls], ['devices', 'devices', 'select'])

    def test_dead_helper_and_stale_com_object_recovered(self):
        for failure in ['dead', 'stale']:
            self.app.last_recovery = float('-inf')
            setattr(self.bridge, failure, True)
            result = self.app.get_state()
            self.assertIsNotNone(result['state'])
            self.assertIsNone(result['connection_error'])
        self.assertEqual(self.bridge.restarts, 2)
        self.assertTrue(all(c['action'] in {'devices', 'state', 'select'} for c in self.bridge.calls))

    def test_device_reordering_matches_path_not_index(self):
        self.bridge.devices = [{'Index': 0, 'Name': 'PTZ Pro 2', 'Path': 'camera-B'}, {**self.device, 'Index': 1}]
        result = self.app.get_state()
        self.assertEqual(result['selected'], 1)
        self.assertEqual(result['state']['Device']['Path'], 'camera-A')

    def test_missing_selected_camera_does_not_switch_to_another(self):
        self.bridge.devices = [{'Index': 0, 'Name': 'PTZ Pro 2', 'Path': 'camera-B'}]
        result = self.app.get_state()
        self.assertIsNone(result['state'])
        self.assertEqual(result['preferred_path'], 'camera-A')
        self.assertEqual([c['action'] for c in self.bridge.calls], ['devices'])

    def test_rediscover_restarts_and_rereads_without_writes(self):
        result = self.app.action({'action': 'rediscover'})
        self.assertIsNotNone(result['state'])
        self.assertEqual(self.bridge.restarts, 1)
        self.assertEqual([c['action'] for c in self.bridge.calls], ['devices', 'select'])

    def test_manual_rediscover_handles_changed_path_when_only_one_camera(self):
        self.bridge.devices = [{**self.device, 'Path':'camera-new-port'}]
        self.assertIsNone(self.app.get_state()['state'])
        result = self.app.action({'action':'rediscover'})
        self.assertEqual(result['state']['Device']['Path'], 'camera-new-port')

    def test_manual_rediscover_does_not_guess_between_multiple_new_cameras(self):
        self.bridge.devices = [{**self.device, 'Path':'camera-B'}, {**self.device, 'Index':1, 'Path':'camera-C'}]
        self.assertIsNone(self.app.action({'action':'rediscover'})['state'])
        self.assertEqual(self.app.preferred_path, 'camera-A')

    def test_startup_without_camera_is_recoverable(self):
        self.bridge.devices = []
        app = App(8765)
        self.assertIsNone(app.state)
        self.bridge.devices = [self.device]
        self.assertIsNotNone(app.get_state()['state'])

    def test_failed_writes_never_replayed_during_recovery(self):
        for action, args in [('set', {'group':'camera', 'id':4, 'value':-10, 'auto':False}),
                             ('jog', {'pan':1, 'tilt':0, 'duration':80, 'speed':1}),
                             ('absolute', {'pan':0, 'tilt':0, 'confirmed':True})]:
            self.bridge.fail_write = action
            self.app.last_recovery = float('-inf')
            with self.assertRaises(ConnectionError):
                self.app.action({'action': action, **args, 'device_path': 'camera-A'})
            self.app.last_recovery = float('-inf')
            self.assertIsNotNone(self.app.get_state()['state'])
            self.assertEqual(sum(c['action'] == action for c in self.bridge.calls), 1)

    def test_select_rejects_stale_inventory_identity(self):
        with self.assertRaises(ValueError):
            self.app.action({'action':'select', 'index':0, 'path':'camera-B'})
        self.assertFalse(any(c['action'] == 'select' for c in self.bridge.calls))

    def test_select_requires_physical_path_not_only_ordinal(self):
        with self.assertRaises(ValueError):
            self.app.action({'action': 'select', 'index': 0})
        self.assertFalse(any(c['action'] == 'select' for c in self.bridge.calls))

    def test_wrong_physical_binding_is_rejected_on_inventory_race(self):
        original = self.bridge.call
        def reordered_reply(request):
            result = original(request)
            if request['action'] in {'state', 'select'}:
                result['Device']['Path'] = 'unexpected-camera-B'
            return result
        self.bridge.call = reordered_reply
        self.assertIsNone(self.app.get_state()['state'])
        self.assertTrue(all(c['action'] in {'devices', 'state', 'select'} for c in self.bridge.calls))


class BridgeTests(unittest.TestCase):
    def test_old_reader_cannot_write_to_replacement_queue(self):
        bridge = Controller.__new__(Controller)
        old_lines, new_lines = queue.Queue(), queue.Queue()
        old_process = Mock(stdout=iter(['old reply\n']))
        bridge.lines = new_lines
        bridge._read(old_process, old_lines)
        self.assertEqual(old_lines.get_nowait(), 'old reply\n')
        self.assertIsNone(old_lines.get_nowait())
        self.assertTrue(new_lines.empty())

    def test_dead_process_and_broken_pipe_raise_connection_error(self):
        bridge = Controller.__new__(Controller)
        bridge.lock = threading.Lock()
        bridge.process = Mock()
        bridge.process.poll.return_value = 1
        with self.assertRaises(ConnectionError):
            bridge.call({'action':'state'})
        bridge.process.poll.return_value = None
        bridge.process.stdin.write.side_effect = BrokenPipeError()
        with self.assertRaises(ConnectionError):
            bridge.call({'action':'state'})

    def test_timeout_kills_owned_helper_and_never_replays_command(self):
        bridge = Controller.__new__(Controller)
        bridge.lock = threading.Lock()
        bridge.process = Mock()
        bridge.process.poll.return_value = None
        bridge.lines = Mock()
        bridge.lines.get.side_effect = queue.Empty()
        with self.assertRaises(ConnectionError):
            bridge.call({'action':'jog'})
        bridge.process.kill.assert_called_once()
        bridge.process.stdin.write.assert_called_once()


class PresetTests(unittest.TestCase):
    """Saved views are exercised against a fake bridge, never a physical camera."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.directory_patch = patch('server.PRESETS', Path(self.directory.name))
        self.directory_patch.start()
        self.addCleanup(self.directory_patch.stop)
        self.app = App.__new__(App)
        self.app.transaction = threading.RLock()
        self.app.connection_revision = 0
        self.app.log = Mock()
        self.app.save_snapshot = Mock(return_value='before-restore.json')
        self.app.state = {
            'Device': {'Name': 'PTZ Pro 2', 'Index': 0, 'Path': 'camera-A'},
            'Controls': [
                {'Group': 'camera', 'Id': 3, 'Name': 'Zoom', 'Supported': True, 'Manual': True, 'Auto': False,
                 'Min': 100, 'Max': 1000, 'Step': 1, 'Value': 100, 'Mode': 'manual'},
                {'Group': 'camera', 'Id': 6, 'Name': 'Focus', 'Supported': True, 'Manual': True, 'Auto': True,
                 'Min': 0, 'Max': 255, 'Step': 1, 'Value': 0, 'Mode': 'manual'},
                {'Group': 'video', 'Id': 7, 'Name': 'WhiteBalance', 'Supported': True, 'Manual': True, 'Auto': True,
                 'Min': 2000, 'Max': 7500, 'Step': 1, 'Value': 2776, 'Mode': 'auto'},
                {'Group': 'video', 'Id': 13, 'Name': 'PowerLineFrequency', 'Supported': True, 'Manual': True, 'Auto': False,
                 'Min': 1, 'Max': 2, 'Step': 1, 'Value': 2, 'Mode': 'unknown'}],
            'Extended': [{'Name': 'AutoExposurePriority', 'Id': 19, 'Readable': True, 'Writable': True, 'Value': 1},
                         {'Name': 'PanRelative', 'Id': 10, 'Readable': True, 'Writable': True, 'Value': 1}],
            'Absolute': {'Readable': True, 'Writable': True, 'Units': 'arcseconds', 'Pan': 7200, 'Tilt': -90000,
                         'PanMin': -324000, 'PanMax': 324000, 'PanStep': 3600,
                         'TiltMin': -144000, 'TiltMax': 108000, 'TiltStep': 3600}}
        self.app.get_state = lambda: None
        self.app.controller = Mock()
        self.app.controller.call.side_effect = self.fake_write

    def fake_write(self, command):
        if command['action'] == 'set':
            control = next(c for c in self.app.state['Controls'] if (c['Group'], c['Id']) == (command['group'], command['id']))
            control['Value'] = command['value']
            if command['id'] != 13:
                control['Mode'] = 'auto' if command['auto'] else 'manual'
            return copy.deepcopy(control)
        if command['action'] == 'extended':
            control = self.app.state['Extended'][0]
            control['Value'] = command['value']
            return copy.deepcopy(control)
        if command['action'] == 'absolute':
            self.app.state['Absolute'].update(Pan=command['pan'], Tilt=command['tilt'])
            return copy.deepcopy(self.app.state['Absolute'])
        self.fail('Unexpected camera command')

    def saved(self):
        return self.app.action({'action': 'save-preset', 'name': 'Full field view', 'device_path': 'camera-A'})['preset']

    def restore(self, preset, **kwargs):
        return self.app.action({'action': 'apply-preset', 'id': preset['id'], 'confirmed': True,
                                'device_path': 'camera-A', **kwargs})

    def test_save_list_and_restart_persistence_are_read_only(self):
        saved = self.saved()
        self.assertEqual(saved['name'], 'Full field view')
        self.assertEqual(saved['state']['Absolute']['Tilt'], -90000)
        replacement = App.__new__(App)
        self.assertEqual(replacement.list_presets()['presets'], [saved])
        self.app.controller.call.assert_not_called()
        self.assertEqual(self.saved()['name'], saved['name'])
        self.assertEqual(len(self.app.list_presets()['presets']), 2, 'Saving must not overwrite the earlier file.')

    def test_invalid_names_and_path_traversal_rejected(self):
        for name in ['', ' ', 'x' * 81, 12, 'bad\nname']:
            with self.assertRaises(ValueError):
                self.app.action({'action': 'save-preset', 'name': name})
        for identifier in ['../snapshot', '', '1234', 'g' * 16, None]:
            with self.assertRaises(ValueError):
                self.app.read_preset(identifier)
        self.app.controller.call.assert_not_called()

    def test_explicit_boolean_confirmation_required(self):
        saved = self.saved()
        for confirmed in [None, False, 'true', 1]:
            with self.assertRaises(ValueError):
                self.restore(saved, confirmed=confirmed)
        self.app.controller.call.assert_not_called()

    def test_matching_view_and_auto_drift_send_no_writes(self):
        saved = self.saved()
        self.app.state['Controls'][2]['Value'] = 3100
        result = self.restore(saved)
        self.assertEqual(result['applied_count'], 0)
        self.assertFalse(result['movement_requested'])
        self.app.controller.call.assert_not_called()

    def test_auto_exposure_gain_drift_is_not_replayed(self):
        self.app.state['Controls'].extend([
            {'Group': 'camera', 'Id': 4, 'Name': 'Exposure', 'Supported': True, 'Manual': True, 'Auto': True,
             'Min': -11, 'Max': -2, 'Step': 1, 'Value': -5, 'Mode': 'auto'},
            {'Group': 'video', 'Id': 9, 'Name': 'Gain', 'Supported': True, 'Manual': True, 'Auto': False,
             'Min': 0, 'Max': 255, 'Step': 1, 'Value': 45, 'Mode': 'manual'}])
        saved = self.saved()
        self.app.state['Controls'][-1]['Value'] = 34
        self.assertEqual(self.restore(saved)['applied_count'], 0)
        self.app.controller.call.assert_not_called()

    def test_explicit_restore_includes_lens_position_and_safe_scalar(self):
        saved = self.saved()
        self.app.state['Controls'][0]['Value'] = 200
        self.app.state['Controls'][1]['Value'] = 12
        self.app.state['Controls'][3]['Value'] = 1
        self.app.state['Extended'][0]['Value'] = 0
        self.app.state['Absolute']['Pan'] = 10800
        result = self.restore(saved)
        self.assertEqual(result['applied_count'], 5)
        self.assertTrue(result['movement_requested'])
        actions = [call.args[0] for call in self.app.controller.call.call_args_list]
        self.assertEqual([c['action'] for c in actions], ['set', 'set', 'set', 'extended', 'absolute'])
        self.assertEqual(actions[-1], {'action': 'absolute', 'pan': 7200, 'tilt': -90000, 'confirmed': True})
        self.assertFalse(any(c.get('id') == 10 for c in actions), 'Relative motor read-back must never be replayed.')
        self.app.save_snapshot.assert_called_once()

    def test_all_ranges_checked_before_any_write(self):
        saved = self.saved()
        self.app.state['Controls'][0]['Value'] = 200
        self.app.state['Controls'][3]['Min'] = 3
        with self.assertRaises(ValueError):
            self.restore(saved)
        self.app.controller.call.assert_not_called()
        self.app.save_snapshot.assert_not_called()

    def test_position_units_off_step_and_missing_lens_block_restore(self):
        for mutate in [lambda s: s['Absolute'].update(PanStep=5000),
                       lambda s: s['Absolute'].update(Units='degrees'),
                       lambda s: s['Controls'].pop(1)]:
            source = self.saved()
            mutate(source['state'])
            with self.assertRaises(ValueError):
                self.app.preset_plan(source)
        self.app.controller.call.assert_not_called()

    def test_stale_selection_and_different_model_block_writes(self):
        saved = self.saved()
        with self.assertRaises(ValueError):
            self.restore(saved, device_path='camera-B')
        self.app.state['Device']['Name'] = 'Other camera'
        with self.assertRaises(ValueError):
            self.restore(saved)
        self.app.controller.call.assert_not_called()

    def test_different_device_requires_extra_confirmation(self):
        saved = self.saved()
        self.app.state['Device']['Path'] = 'camera-B'
        with self.assertRaises(ValueError):
            self.restore(saved, device_path='camera-B')
        result = self.restore(saved, device_path='camera-B', allow_different_device=True)
        self.assertEqual(result['applied_count'], 0)
        self.app.controller.call.assert_not_called()

    def test_failed_write_stops_once_without_retry_rollback_or_movement(self):
        saved = self.saved()
        self.app.state['Controls'][0]['Value'] = 200
        self.app.state['Controls'][1]['Value'] = 12
        self.app.state['Absolute']['Pan'] = 10800
        self.app.controller.call.side_effect = ConnectionError('Timed out')
        with self.assertRaisesRegex(ConnectionError, 'No command was repeated or rolled back'):
            self.restore(saved)
        self.app.controller.call.assert_called_once()
        self.assertIsNone(self.app.state)

    def test_bad_readback_stops_before_motor_command(self):
        saved = self.saved()
        self.app.state['Controls'][0]['Value'] = 200
        self.app.state['Absolute']['Pan'] = 10800
        self.app.controller.call.side_effect = None
        self.app.controller.call.return_value = {'Supported': True, 'Value': 200, 'Mode': 'manual'}
        with self.assertRaisesRegex(ConnectionError, 'read-back'):
            self.restore(saved)
        self.app.controller.call.assert_called_once()

    def test_camera_writes_reject_missing_or_stale_identity(self):
        for action, args in [('set', {'group': 'camera', 'id': 3, 'value': 100, 'auto': False}),
                             ('jog', {'pan': 1, 'tilt': 0, 'duration': 80, 'speed': 1}),
                             ('stop', {}), ('extended', {'id': 19, 'value': 0}),
                             ('absolute', {'pan': 7200, 'tilt': -90000, 'confirmed': True}),
                             ('save-preset', {'name': 'Wrong camera'}), ('snapshot', {'label': 'wrong'})]:
            for path in [None, 'camera-B']:
                with self.assertRaisesRegex(ValueError, 'Camera selection changed'):
                    self.app.action({'action': action, **args, 'device_path': path})
        self.app.controller.call.assert_not_called()

    def test_native_preview_bound_to_control_path_and_token_scoped(self):
        fake = Mock()
        fake.closed = False
        fake.path = 'camera-A'
        fake.token = 'preview-token-A'
        fake.info.return_value = {'device_path': 'camera-A', 'token': fake.token, 'error': None}
        with patch('server.NativePreview', return_value=fake) as factory:
            with self.assertRaises(ValueError):
                self.app.action({'action': 'preview-start', 'device_path': 'camera-B'})
            factory.assert_not_called()
            result = self.app.action({'action': 'preview-start', 'device_path': 'camera-A'})
            factory.assert_called_once_with('camera-A')
            self.assertEqual(result['device_path'], 'camera-A')
            self.app.action({'action': 'preview-start', 'device_path': 'camera-A'})
            factory.assert_called_once()
            self.app.action({'action': 'preview-stop', 'token': 'stale-token'})
            fake.close.assert_not_called()
            self.app.action({'action': 'preview-stop', 'token': fake.token})
            fake.close.assert_called_once()
            self.assertIsNone(self.app.preview)
        self.app.controller.call.assert_not_called()

    def test_disconnect_releases_only_owned_preview(self):
        self.app.preview = Mock()
        owned = self.app.preview
        self.app.disconnected('unplugged')
        owned.close.assert_called_once()
        self.assertIsNone(self.app.preview)

    def test_dshow_source_preserves_physical_identifier(self):
        source = '@device:pnp:\\\\?\\usb#vid_046d&pid_085f&mi_00#7&camera-A&0&0000#{capture}\\global'
        self.assertEqual(dshow_name(source), source.replace(':', '_'))
        for value in ['PTZ Pro 2', '0', '@device:pnp:other', None]:
            with self.assertRaises(ValueError):
                dshow_name(value)


if __name__ == '__main__':
    unittest.main()
