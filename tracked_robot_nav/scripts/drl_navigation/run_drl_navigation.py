#!/usr/bin/env python3

"""
Main script to run the DRL navigation system.
"""

import sys
import os

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drl_navigation.test_drl_navigation import main as test_drl_navigation

if __name__ == "__main__":
    test_drl_navigation()
