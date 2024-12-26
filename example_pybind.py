import os, sys

sys.path.append("./")
# add the path to the pybind11 module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "./build/")))

from u_shaped_turn import drive, planHybridAStar
import numpy as np

print("----test drive----")
pts = np.array([[]])
start_pose = np.array([10.0, 0.0, 0.0])
path = drive(pts, start_pose)
print(path)


print("----test planHybridAStar----")
width = 10
height = 15
nStart = np.array([1, 1, 0, 0, 0])
nGoal = np.array([5, 1, 0, 0, 0])
path = planHybridAStar(width, height, nStart, nGoal)
print(path)