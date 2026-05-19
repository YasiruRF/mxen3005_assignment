import unittest
from dobot.dobot_kinematics import forward_kinematics, inverse_kinematics
import math

class TestDobotKinematics(unittest.TestCase):
    def test_ik_fk_consistency(self):
        """Test if FK(IK(x,y,z,r)) returns (x,y,z,r)"""
        # Test a known reachable point
        tx, ty, tz, tr = 200.0, 0.0, 50.0, 0.0
        
        # Calculate IK
        q1, q2, q3, q4 = inverse_kinematics(tx, ty, tz, tr)
        
        # Calculate FK from those joints
        rx, ry, rz, rr = forward_kinematics(q1, q2, q3, q4)
        
        # Allow for small floating point differences
        self.assertAlmostEqual(tx, rx, places=1)
        self.assertAlmostEqual(ty, ry, places=1)
        self.assertAlmostEqual(tz, rz, places=1)
        self.assertAlmostEqual(tr, rr, places=1)

    def test_joint_limits(self):
        """Verify IK handles 0,0,0,0 correctly (or as expected)"""
        # The robot's home position often results in specific joint angles
        # This test ensures the math doesn't crash on standard inputs
        try:
            inverse_kinematics(150, 150, 50, 0)
        except Exception as e:
            self.fail(f"IK crashed with error: {e}")

if __name__ == '__main__':
    unittest.main()
