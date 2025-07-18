#! /usr/bin/env python3

'''
This file should contain the main application logic for CV2X Crack Detection.
'''

'''
Import lists as three groups, in each group the imports should be ordered alphabetically.
group 1: standard library imports
group 2: third-party library imports installed via pip
group 3: local application/library specific imports
'''


import argparse
import os
import sys
import time

import numpy as np

# from utility import ...


class CV2XCrackDetection(Node):
    def __init__(self, api, acc, test_type, ts_filename):
        super().__init__('cv2x_crack_detection')

    def some_method(self):
        # Implement the method logic here
        pass
   
   

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Starting CV2X Crack Detection Application")

    # should be as abstract as possible,
    # can be translated to a few lines of pseudo code
