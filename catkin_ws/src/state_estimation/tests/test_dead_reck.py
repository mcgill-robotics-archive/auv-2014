import dead_reck
from unittest import TestCase
import math

class ClockMock:
    def __init__(self, t=0):
        self.time = t

    def set_time(self, t):
        self.time = t

    def get_time(self):
        return self.time

class TestDeadReck(TestCase):


    def setUp(self):
        self.dead_reck = dead_reck.dead_reck(ClockMock())


    def test_sanity(self):
        self.assertEqual(4, 2+2, "2 + 2 != 4")

    def test_updateCV_hasNoTargetToStart(self):
        self.assertFalse(self.dead_reck.getState()['hasTarget'])

    def test_updateCV_hasTarget(self):
        self.dead_reck.updateCV(True,1,1,0,0,0,0)
        self.assertTrue(self.dead_reck.getState()['hasTarget'])

    def test_updateCV_hasNoTarget(self):
        self.dead_reck.updateCV(False,1,1,0,0,0,0)
        self.assertFalse(self.dead_reck.getState()['hasTarget'])

    def test_updateCV_changeID(self):
        self.dead_reck.updateCV(True,1,1,0,0,0,0)
        self.dead_reck.updateCV(True,2,1,0,0,0,0)
        self.assertEqual(2, self.dead_reck.getState()["targetID"])

    def test_updateOrientation_orientationChange(self):
        self.dead_reck.updateOrientation(0)
        self.dead_reck.updateCV(True,1,1,0,0,0,0)
        self.dead_reck.updateOrientation(math.pi/2)
        self.assertAlmostEqual(0, self.dead_reck.getState()['x'], 10)
        self.assertAlmostEqual(-1, self.dead_reck.getState()['y'], 10)
        self.assertAlmostEqual(math.pi/2, self.dead_reck.getState()['yawOfTarget'])

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('test_state_estimation', 'test_dead_reck', TestDeadReck)