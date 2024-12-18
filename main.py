import os, sys
sys.path.append("./")
# add the path to the pybind11 module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './build/')))

import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from abc import ABC, abstractclassmethod
from easydict import EasyDict

from scene import getObjects

class RoadCurbPlotter:
    def __init__(self, data: EasyDict):
        self.data = data
    
    def render(self, ax):
        for curb in self.data:
            ax.plot([curb.start.x, curb.end.x], [curb.start.y, curb.end.y])

class DrivePathPlotter:
    def __init__(self, data: EasyDict):
        self.data = data
    
    def render(self, ax):
        xs = []
        ys = []
        yaws = []
        for pt in self.data:
            xs.append(pt.x)
            ys.append(pt.y)
            yaws.append(pt.yaw)
        ax.plot(xs, ys)

# class StartPointPlotter:
#     def __init__(self, data: EasyDict):
#         self.data = data
    
#     def render(self, ax):


if __name__ == "__main__":
    print("hello world")
    import json
    
    uturn_input = EasyDict()
    file_path = "./resources/uturn_input.json"
    with open(file_path, "r") as f:
        uturn_json = json.load(f)
        uturn_input = EasyDict(uturn_json)
    
    print(uturn_input.road_curbs[0].start)

    fig, ax = plt.subplots()
    rc_plotter = RoadCurbPlotter(uturn_input.road_curbs)
    rc_plotter.render(ax)
    drive_path_plotter = DrivePathPlotter(uturn_input.drive_path)
    drive_path_plotter.render(ax)

    # ax.scatter(uturn_input.start_point.x, uturn_input.start_point.y, 'x')
    # go_points = EasyDict({'xs': [], 'ys': []})
    # for pt in uturn_input.go_points:
    #     go_points.xs.append(pt.x)
    #     go_points.ys.append(pt.y)
    # ax.scatter(go_points.xs, go_points.ys, '+')

    plt.savefig('main.png')
    plt.show()