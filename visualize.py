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
from drive_path import DrivePath, DrivePathReal, UShapedTurnModelInterface

class UShapeTurnModelReal(UShapedTurnModelInterface):
    def __init__(self, drive_path: DrivePath):
        self.drive_path = drive_path
    
    def decode(self, current_pose):
        steer_map = [0.5, 0. -0.5]
        steer_idx = 1
        drive_path_poses = self.drive_path.getPoses()
        if drive_path_poses[0].yaw > 1e-2:
            steer_idx = 0
        elif drive_path_poses[0].yaw < -1e-2:
            steer_idx = 2
        else:
            steer_idx = 1
        
        return steer_map[steer_idx]

def planUTurn(plotter_impl: PlotterImplInterface, uturn_input: EasyDict):
    # drawUTurnInput(plotter_impl, uturn_input)
    start_pose = uturn_input.start_point
    
    trajs_drawers = []
    for go_point in uturn_input.go_points:
        end_pose = go_point
        drive_path = DrivePathReal(uturn_input.drive_path)
        model = UShapeTurnModelReal(drive_path)
        traj = Plan(start_pose, end_pose, model)
        trajs_drawers.append(LineDrawer(traj))
    
    for traj_drawer in trajs_drawers:
        traj_drawer.setPlotter(plotter_impl)
        traj_drawer.show()
    plotter_impl.show()

import json
def parseInput(file_path):
    uturn_input = EasyDict()
    with open(file_path, "r") as f:
        uturn_json = json.load(f)
        uturn_input = EasyDict(uturn_json)
    
    return uturn_input

# for bokeh only
from bokeh_plotter_impl import BokehPlotter
def bokehPlotUTurnInput():
    plotter_impl = BokehPlotter()
    uturn_input = parseInput("./resources/uturn_input_12191203.json")
    drawUTurnInput(plotter_impl, uturn_input)
    plotter_impl.show()

bokehPlotUTurnInput()

if __name__ == "__main__":
    import math
    import sys

    uturn_input = parseInput(sys.argv[1])

    print(uturn_input.road_curbs[0].start)
    
    plotter_impl = MatPlotterImpl()
    drawUTurnInput(plotter_impl, uturn_input)
    # planUTurn(plotter_impl, uturn_input)
    
    plotter_impl.show()