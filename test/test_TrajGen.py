#! usr/bin/env python
import unittest
import os
import sys
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..')))
import src
from src.trajectory_generation import InsideDistThresh

class MyTest(unittest.TestCase):
    def test_1(self):
        pos_start = [0.0,0.0]
        pos_end = [1.0, 1.0]
        DIST_THRE = 1.0
        self.assertTrue( InsideDistThresh( pos_start, pos_end, DIST_THRE)==True, "Should be False")


unittest.main()

