import importlib.util
import pathlib
import threading
import unittest

PATH = pathlib.Path(__file__).resolve().parents[1] / 'rover' / 'rover_keyboard.py'
spec = importlib.util.spec_from_file_location('rover_keyboard', PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Transport:
    live = False

    def __init__(self):
        self.sent = []
        self.fail = False

    def send(self, pair):
        self.sent.append(pair)
        if self.fail:
            raise OSError('simulated Wi-Fi loss')


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.t = Transport()
        self.now = 1.0
        self.c = m.Controller(self.t, clock=lambda: self.now)
        self.token = self.c.new_session()
        self.seq = 0

    def cmd(self, action, keys=()):
        self.seq += 1
        return self.c.command(self.token, self.seq, action, list(keys))

    def arm(self):
        self.c.step()
        self.assertTrue(self.cmd('arm')['armed'])

    def test_default_disarmed_and_arm_needs_stop_response(self):
        self.assertFalse(self.cmd('arm')['armed'])
        self.arm()
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)

    def test_directions_and_release(self):
        self.arm()
        for key, pair in m.DIRECTIONS.items():
            self.cmd('state', [key])
            self.c.step()
            self.assertEqual(self.t.sent[-1], pair)
            self.cmd('state')
            self.c.step()
            self.assertEqual(self.t.sent[-1], m.STOP)

    def test_repeats_and_browser_lease(self):
        self.arm()
        self.cmd('state', ['KeyW'])
        self.c.step()
        self.now += .1
        self.c.step()
        self.assertEqual(self.t.sent[-2:], [(m.POWER, m.POWER)] * 2)
        self.now += .36
        self.c.step()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.t.sent[-1], m.STOP)
        self.cmd('state', ['KeyW'])
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)

    def test_late_packet_cannot_renew_expired_lease(self):
        self.arm()
        self.now += m.LEASE
        self.assertFalse(self.cmd('state', ['KeyW'])['armed'])

    def test_emergency_stop_latches_and_rearm_requires_delivered_stop(self):
        self.arm()
        self.cmd('state', ['KeyW'])
        self.c.step()
        self.cmd('stop')
        self.assertFalse(self.cmd('arm')['armed'])
        self.c.step()
        self.assertTrue(self.cmd('arm')['armed'])
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)

    def test_old_move_cannot_undo_stop(self):
        self.arm()
        old = self.seq + 1
        self.cmd('state', ['KeyW'])
        self.cmd('stop')
        self.c.command(self.token, old, 'state', ['KeyW'])
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)
        self.assertFalse(self.c.armed)

    def test_network_failure_latches_and_only_retries_stop(self):
        self.arm()
        self.cmd('state', ['KeyW'])
        self.t.fail = True
        self.c.step()
        self.assertFalse(self.c.armed)
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)
        self.assertFalse(self.cmd('arm')['armed'])
        self.t.fail = False
        self.c.step()
        self.assertFalse(self.c.armed)
        self.assertTrue(self.cmd('arm')['armed'])

    def test_new_page_invalidates_old_page_and_disarms(self):
        self.arm()
        self.cmd('state', ['KeyW'])
        self.c.new_session()
        with self.assertRaises(ValueError):
            self.cmd('state', ['KeyW'])
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)

    def test_failed_idle_stop_blocks_arming(self):
        self.c.step()
        self.t.fail = True
        self.c.step()
        self.assertFalse(self.cmd('arm')['armed'])

    def test_shutdown_sends_stop_after_moving(self):
        # Use a real clock because close() has a bounded wait.
        c = m.Controller(self.t)
        token = c.new_session()
        c.step()
        c.command(token, 1, 'arm', [])
        c.command(token, 2, 'state', ['KeyW'])
        c.step()
        c.start()
        c.close()
        self.assertFalse(c.worker.is_alive())
        self.assertEqual(self.t.sent[-1], m.STOP)
        self.assertFalse(c.armed)

    def test_conflicting_keys_stop(self):
        self.arm()
        self.cmd('state', ['KeyW', 'KeyD'])
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)

    def test_stop_during_inflight_movement(self):
        self.arm()
        self.cmd('state', ['KeyW'])
        entered, release = threading.Event(), threading.Event()
        original = self.t.send

        def blocked_send(pair):
            entered.set()
            release.wait(1)
            original(pair)

        self.t.send = blocked_send
        th = threading.Thread(target=self.c.step)
        th.start()
        self.assertTrue(entered.wait(1))
        self.cmd('stop')
        self.assertFalse(self.cmd('arm')['armed'])
        release.set()
        th.join(1)
        self.assertFalse(th.is_alive())
        self.c.step()
        self.assertEqual(self.t.sent[-1], m.STOP)
        self.assertFalse(self.c.armed)

    def test_bad_inputs_do_not_change_state(self):
        self.arm()
        for seq, action, keys in [(True, 'state', []), (3, 'bad', []),
                                   (3, 'state', ['KeyX']), (3, 'state', 'KeyW'),
                                   (3, 'state', [['KeyW']])]:
            with self.assertRaises(ValueError):
                self.c.command(self.token, seq, action, keys)
        self.assertEqual(self.c.pair, m.STOP)


if __name__ == '__main__':
    unittest.main()
