import numpy as np
from easydict import EasyDict
import math

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

    def getProjectionIdx(self, pose):
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
        return velocity_map[projection_segment_idx], steer_map[projection_segment_idx]

if __name__=="__main__":
    dp = DrivePath()
    print("expected steer is (1.0, 0.0), result is: {}".format(dp.Query(EasyDict(x=0.0, y=-1.0, yaw=0))))
    print("expected steer is (1.0, 0.5), result is: {}".format(dp.Query(EasyDict(x=0.0, y=1.0, yaw=0))))