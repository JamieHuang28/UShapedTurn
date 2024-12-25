import matplotlib.pyplot as plt
from abc import ABC, abstractmethod

class PlotterImplInterface(ABC):
    @abstractmethod
    def plotLine(self, line):
        pass
    
    @abstractmethod
    def plotPose(self, pose):
        pass
    
    @abstractmethod
    def plotMultiLines(self, lines):
        pass
    
    @classmethod
    @abstractmethod
    def show(self):
        pass

class DrawerAbstract(ABC):
    def __init__(self):
        self.plotter = None
    
    def setPlotter(self, plotter: PlotterImplInterface):
        self.plotter = plotter
    
    @abstractmethod
    def draw(self):
        pass

class DrawerAbstract(ABC):
    def __init__(self):
        self.plotter = None
    
    def setPlotter(self, plotter: PlotterImplInterface):
        self.plotter = plotter
    
    @abstractmethod
    def draw(self):
        pass

# for example only
class PoseDrawer(DrawerAbstract):
    def __init__(self, pose):
        self.pose = pose
    
    def draw(self):
        self.plotter.plotPose(self.pose)

class LineDrawer(DrawerAbstract):
    def __init__(self, line):
        self.line = line
    
    def draw(self):
        self.plotter.plotLine(self.line)

class PrintPlotter(PlotterImplInterface):
    def __init__(self):
        pass
    
    def _plot(self, data):
        print(data)
    
    def plotLine(self, line):
        self._plot(line)
    
    def plotPose(self, pose):
        self._plot(pose)
    
    def plotMultiLines(self, lines):
        self._plot(lines)

class DoublePrintPlotter(PrintPlotter):
    def __init__(self):
        pass
    
    def _plot(self, data):
        print(data, data)

if __name__ == "__main__":
    pose_data = "PoseData"
    line_data = "LineData"
    
    print_plotter = PrintPlotter()
    double_print_plotter = DoublePrintPlotter()
    
    pose_drawer = PoseDrawer(pose_data)
    pose_drawer.setPlotter(print_plotter)
    line_drawer = LineDrawer(line_data)
    line_drawer.setPlotter(print_plotter)
    
    pose_drawer.draw()
    line_drawer.draw()
    
    pose_drawer.setPlotter(double_print_plotter)
    pose_drawer.draw()