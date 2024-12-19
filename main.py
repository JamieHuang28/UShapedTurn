import os, sys
sys.path.append("./")
# add the path to the pybind11 module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './build/')))

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from abc import ABC, abstractclassmethod
from easydict import EasyDict

if __name__ == "__main__":
    print("hello world")