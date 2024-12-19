import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
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
    import json
    import math
    
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

    ax.arrow(uturn_input.start_point.x, uturn_input.start_point.y, math.cos(uturn_input.start_point.yaw), math.sin(uturn_input.start_point.yaw),
             width=.1, length_includes_head=True, color="C2")
    # go_points = EasyDict({'xs': [], 'ys': []})
    for go_point in uturn_input.go_points:
        ax.arrow(go_point.x, go_point.y, math.cos(go_point.yaw), math.sin(go_point.yaw),
             width=.1, length_includes_head=True, color="r")
    # ax.scatter(go_points.xs, go_points.ys, '+')

    plt.savefig('visualize.png')
    plt.show()