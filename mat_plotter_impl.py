import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

import math
from easydict import EasyDict
from drawer_plotter_abstract import PlotterImplInterface

class MatLinePlotter:
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

class MatMultiLinesPlotter:
    def __init__(self, data: EasyDict):
        self.data = data

    def render(self, ax):
        for curb in self.data:
            ax.plot([curb.start.x, curb.end.x], [curb.start.y, curb.end.y])

class MatPosePlotter:
    def __init__(self, data: EasyDict):
        self.start_point = data
    
    def render(self, ax):
        ax.arrow(
            self.start_point.x,
            self.start_point.y,
            math.cos(self.start_point.yaw),
            math.sin(self.start_point.yaw),
            width=0.1,
            length_includes_head=True
        )

class MatPlotterImpl(PlotterImplInterface):
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
    
    def plotLine(self, line):
        mat_line_plotter = MatLinePlotter(line)
        mat_line_plotter.render(self.ax)
    
    def plotPose(self, pose):
        mat_pose_plotter = MatPosePlotter(pose)
        mat_pose_plotter.render(self.ax)
    
    def plotMultiLines(self, lines):
        mat_line_plotter = MatMultiLinesPlotter(lines)
        mat_line_plotter.render(self.ax)