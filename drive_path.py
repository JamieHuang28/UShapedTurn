import numpy as np
from easydict import EasyDict
import math
from bokeh.palettes import Spectral7

from abc import ABC, abstractmethod

class DrivePathInterface(ABC):
    @abstractmethod
    def numPoints(self):
        pass
    
    @abstractmethod
    def getPoses(self):
        xs, ys, yaws = self.get()
        return [EasyDict(x=a, y=b, yaw=c) for (a, b, c) in zip(xs, ys, yaws)]

    @abstractmethod
    def getPointsAndArrow(self):
        pass

    @abstractmethod
    def getSegments(self):
        pass

class DrivePathFake(DrivePathInterface):
    def __init__(self):
        #          /\
        #        kHeight
        #       /      \
        # return_point  turn_point
        #      |        |
        # kParallel   kParallel
        #      |        |
        self.kHeight = 10.0
        self.kParallel = 5.0
        self.turn_point = EasyDict(x=6.0, y=0.0, yaw=np.pi / 2)
        self.return_point = EasyDict(x=-6.0, y=0.0, yaw=-np.pi / 2)
        self.poses = self.getPoses()
    
    def numPoints(self):
        return 5

    def get(self):
        xs = [
            self.turn_point.x,
            self.turn_point.x,
            (self.turn_point.x + self.return_point.x) / 2.0,
            self.return_point.x,
            self.return_point.x,
        ]
        ys = [
            self.turn_point.y - self.kParallel,
            self.turn_point.y,
            (self.turn_point.y + self.return_point.y) / 2.0 + self.kHeight,
            self.return_point.y,
            self.return_point.y - self.kParallel,
        ]
        yaws = [
            self.turn_point.yaw,
            self.turn_point.yaw,
            np.pi,
            self.return_point.yaw,
            self.return_point.yaw,
        ]
        return xs, ys, yaws
    
    def getPoses(self):
        xs, ys, yaws = self.get()
        return [EasyDict(x=a, y=b, yaw=c) for (a, b, c) in zip(xs, ys, yaws)]

    def getPointsAndArrow(self):
        xs, ys, yaws = self.get()

        return dict(
            x_start=xs,
            y_start=ys,
            x_end_arrow=[a + math.cos(b) for (a, b) in zip(xs, yaws)],
            y_end_arrow=[a + math.sin(b) for (a, b) in zip(ys, yaws)],
        )

    def getSegments(self):
        xs, ys, yaws = self.get()

        start_end_xs = [a for a in zip(xs[0:-1], xs[1:])]
        start_end_ys = [a for a in zip(ys[0:-1], ys[1:])]
        colors = Spectral7[::-1][0 : len(start_end_xs)]
        return dict(segment_xs=start_end_xs, segment_ys=start_end_ys, colors=colors)

    def getTurnPoint(self):
        return self.turn_point

class DrivePath(DrivePathInterface):
    def __init__(self, data):
        self.data = data
    
    def numPoints(self):
        return len(self.data)
    
    def getPointsAndArrow(self):
        return []

    def getPoses(self):
        return self.data
    
    def getSegments(self):
        start_end_xs = [(pose[0].x, pose[1].x) for pose in zip(self.data[0:-1], self.data[1:])]
        start_end_ys = [(pose[0].y, pose[1].y) for pose in zip(self.data[0:-1], self.data[1:])]
        colors = [Spectral7[::-1][i%len(Spectral7)] for i in range(len(self.data))]
        return dict(segment_xs=start_end_xs, segment_ys=start_end_ys, colors=colors)
            
if __name__ == "__main__":
    pass
