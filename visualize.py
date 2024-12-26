import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

import numpy as np
from easydict import EasyDict

from bokeh.palettes import Spectral7

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

from planner import Plan
from drive_path import DrivePath
from model import UShapedTurnModel

def planUTurn(plotter_impl: PlotterImplInterface, uturn_input: EasyDict):
    drawUTurnInput(plotter_impl, uturn_input)
    start_pose = uturn_input.start_point
    
    trajs_drawers = []
    for end_pose in uturn_input.go_points:
        drive_path = DrivePath(uturn_input.drive_path)
        model = UShapedTurnModel(drive_path)
        traj = Plan(start_pose, end_pose, model)
        print("traj length of target {} is {}".format(end_pose, len(traj)))
        trajs_drawers.append(LineDrawer(traj))
    
    for traj_drawer in trajs_drawers:
        traj_drawer.setPlotter(plotter_impl)
        traj_drawer.draw()
    plotter_impl.show()

from utils.file import parseInput

# for bokeh only
from bokeh_plotter_impl import BokehPlotter
def bokehPlotUTurnInput():
    plotter_impl = BokehPlotter()
    uturn_input = parseInput("./resources/uturn_input_12191203.json")
    drawUTurnInput(plotter_impl, uturn_input)
    plotter_impl.show()

bokehPlotUTurnInput()

def testDrawInput(uturn_input):
    plotter_impl = MatPlotterImpl()
    drawUTurnInput(plotter_impl, uturn_input)
    plotter_impl.show()

if __name__ == "__main__":
    import math
    import sys

    uturn_input = parseInput(sys.argv[1])
    
    plotter_impl = MatPlotterImpl()
    planUTurn(plotter_impl, uturn_input)
    
    plotter_impl.show()