import unittest
import math
import sys
import os

# Add parent to path so we can import sentry_sensors before it's installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sentry_sensors.imu_node import ImuNode


class TestQuaternionNormalize(unittest.TestCase):
    """Test the quaternion normalization helper."""

    def test_unit_quaternion_unchanged(self):
        q = [1.0, 0.0, 0.0, 0.0]
        result = ImuNode._normalize_quaternion_static(q)
        self.assertEqual(result, [1.0, 0.0, 0.0, 0.0])

    def test_zero_quaternion_fallback(self):
        q = [0.0, 0.0, 0.0, 0.0]
        result = ImuNode._normalize_quaternion_static(q)
        self.assertEqual(result, [1.0, 0.0, 0.0, 0.0])

    def test_large_norm_normalized(self):
        q = [2.0, 0.0, 0.0, 0.0]
        result = ImuNode._normalize_quaternion_static(q)
        self.assertEqual(result, [1.0, 0.0, 0.0, 0.0])

    def test_non_unit_normalized(self):
        q = [0.5, 0.5, 0.5, 0.5]
        result = ImuNode._normalize_quaternion_static(q)
        norm = math.sqrt(sum(x * x for x in result))
        self.assertAlmostEqual(norm, 1.0, places=6)


class TestCovarianceStructure(unittest.TestCase):
    """Test covariance matrix helpers."""

    def test_build_covariance_3x3(self):
        flat = [0.0005, 0.0, 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0, 0.0008]
        matrix = ImuNode._build_covariance_matrix(flat)
        self.assertEqual(len(matrix), 9)
        self.assertEqual(matrix[0], 0.0005)
        self.assertEqual(matrix[4], 0.0005)
        self.assertEqual(matrix[8], 0.0008)


if __name__ == '__main__':
    unittest.main()
