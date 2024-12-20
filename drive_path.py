import numpy as np
from easydict import EasyDict
import math
from bokeh.palettes import Spectral7

class DrivePath:
    def __init__(self):
        #          /\
        #        kHeight
        #       /      \
        # return_point  turn_point
        #      |        |
        # kParallel   kParallel
        #      |        |
        self.turn_point = EasyDict(x=6.0, y=0.0, yaw=np.pi/2)
        self.return_point = EasyDict(x=-6.0, y=0.0, yaw=-np.pi/2)
        self.poses = []
        self.kHeight = 10.0
        self.kParallel = 5.0
    
    def get(self):
        xs = [self.turn_point.x, self.turn_point.x, (self.turn_point.x + self.return_point.x) / 2.0, self.return_point.x, self.return_point.x]
        ys = [self.turn_point.y-self.kParallel, self.turn_point.y, (self.turn_point.y + self.return_point.y) / 2.0 + self.kHeight, self.return_point.y, self.return_point.y-self.kParallel]
        yaws = [self.turn_point.yaw, self.turn_point.yaw, np.pi, self.return_point.yaw, self.return_point.yaw]
        self.poses = [EasyDict(x=a, y=b, yaw=c) for (a,b,c) in zip(xs, ys, yaws)]
        return xs, ys, yaws

    def getPointsAndArrow(self):
        xs, ys, yaws = self.get()
        
        return dict(x_start=xs, y_start=ys, x_end_arrow=[a + math.cos(b) for (a,b) in zip(xs, yaws)], y_end_arrow=[a + math.sin(b) for (a,b) in zip(ys, yaws)])
    
    def getSegments(self):
        xs, ys, yaws = self.get()
        
        start_end_xs = [a for a in zip(xs[0:-1], xs[1:])]
        start_end_ys = [a for a in zip(ys[0:-1], ys[1:])]
        colors = Spectral7[::-1][0:len(start_end_xs)]
        return dict(segment_xs=start_end_xs, segment_ys=start_end_ys, colors=colors)
        

    def getProjectionIdx(self, pose):
        # temporarily a very simple implementation
        if pose.y < self.turn_point.y:
            return 0
        return 1
    
    def Query(self, pose):
        """
        'Query' is the concept of Transformer
        """
        projection_segment_idx = self.getProjectionIdx(pose)
        steer_map = [0, 0.5, 0.5, 0]
        velocity_map = [1.0, 1.0, 1.0, 1.0]
        return velocity_map[projection_segment_idx], steer_map[projection_segment_idx], projection_segment_idx

if __name__=="__main__":
    dp = DrivePath()
    print("expected steer is (1.0, 0.0, 0), result is: {}".format(dp.Query(EasyDict(x=0.0, y=-1.0, yaw=0))))
    print("expected steer is (1.0, 0.5, 1), result is: {}".format(dp.Query(EasyDict(x=0.0, y=1.0, yaw=0))))