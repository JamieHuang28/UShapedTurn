import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

import numpy as np
from easydict import EasyDict

from drawer_plotter_abstract import PlotterImplInterface, DrawerAbstract
from mat_plotter_impl import MatPlotterImpl

class LineDrawer(DrawerAbstract):
    def __init__(self, data: EasyDict):
        self.data = data
    
    def draw(self):
        self.plotter.plotLine(self.data)

class MultiLinesDrawer(DrawerAbstract):
    def __init__(self, data: EasyDict):
        self.data = data
    
    def draw(self):
        self.plotter.plotMultiLines(self.data)

class PoseDrawer(DrawerAbstract):
    def __init__(self, data: EasyDict):
        self.data = data
    
    def draw(self):
        self.plotter.plotPose(self.data)

def drawUTurnInput(plotter_impl: PlotterImplInterface, uturn_input: EasyDict):
    drawers = []
    drawers.append(MultiLinesDrawer(uturn_input.road_curbs))
    drawers.append(LineDrawer(uturn_input.drive_path))
    drawers.append(PoseDrawer(uturn_input.start_point))
    
    for go_point in uturn_input.go_points:
        go_point_drawer = PoseDrawer(go_point)
        drawers.append(go_point_drawer)
    
    for drawer in drawers:
        drawer.setPlotter(plotter_impl)
        drawer.draw()
    plotter_impl.show()

if __name__ == "__main__":
    import json
    import math
    import sys

    uturn_input = EasyDict()
    file_path = sys.argv[1]
    with open(file_path, "r") as f:
        uturn_json = json.load(f)
        uturn_input = EasyDict(uturn_json)

    print(uturn_input.road_curbs[0].start)
    
    plotter_impl = MatPlotterImpl()
    drawUTurnInput(plotter_impl, uturn_input)