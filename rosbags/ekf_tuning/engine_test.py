"""Numerical adapter checks; run with python3 -m unittest engine_test.py."""

from concurrent.futures import ThreadPoolExecutor
import unittest

import numpy as np

from engine import Engine


def straight_events(speed=.2, yaw_rate=0.):
    times = np.arange(0., 5., .02)
    wheel = np.zeros((len(times), 8))
    wheel[:, 0] = times
    wheel[:, 2] = speed
    imu = np.zeros_like(wheel)
    imu[:, 0] = times
    imu[:, 1] = 1
    imu[:, 4] = yaw_rate * times
    imu[:, 5] = yaw_rate
    result = np.vstack((wheel, imu))
    return result[np.argsort(result[:, 0], kind="stable")]


class EngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.engine = Engine()

    def test_constant_velocity_integrates_in_metres(self):
        events = straight_events()
        states = self.engine.run(events)
        np.testing.assert_allclose(states[:, 1], events[:, 0] * .2, atol=1e-10)
        np.testing.assert_allclose(states[:, 2:4], 0., atol=1e-12)

    def test_relative_yaw_and_wrap_preserve_turn(self):
        events = straight_events(speed=0., yaw_rate=1.2)
        events[events[:, 1] == 1, 4] += 2.4
        events[:, 4] = np.arctan2(np.sin(events[:, 4]), np.cos(events[:, 4]))
        states = self.engine.run(events, mask=[1,1,0,0,1,1,0,0])
        angle_error = states[:,3] - events[:,0] * 1.2
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        self.assertLess(np.max(np.abs(angle_error)), .01)

    def test_threads_have_independent_states(self):
        events = straight_events(yaw_rate=.1)
        expected = self.engine.run(events)
        with ThreadPoolExecutor(max_workers=4) as pool:
            outputs = list(pool.map(self.engine.run, [events] * 8))
        for output in outputs:
            np.testing.assert_array_equal(output, expected)

    def test_bad_covariance_and_order_rejected(self):
        with self.assertRaises(ValueError):
            self.engine.run(straight_events()[::-1])
        with self.assertRaises(ValueError):
            self.engine.run(straight_events(), r_diag=[-1.] * 8)
        with self.assertRaises(ValueError):
            self.engine.run(np.zeros((5, 7)))


if __name__ == "__main__":
    unittest.main()
