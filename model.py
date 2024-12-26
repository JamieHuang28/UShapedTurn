from abc import ABC, abstractmethod
from easydict import EasyDict
import numpy as np
import math

from drive_path import DrivePathInterface, DrivePathFake, DrivePath

class UShapedTurnModelInterface(ABC):
    @abstractmethod
    def decode(self, current_pose):
        pass

class UShapedTurnModelFake(UShapedTurnModelInterface):
    def __init__(self, drive_path: DrivePathFake):
        self.turn_point = drive_path.getTurnPoint()
    
    def _getProjectionIdx(self, pose):
        # temporarily a very simple implementation
        if pose.y < self.turn_point.y:
            return 0
        return 1
    
    def decode(self, current_pose):
        projection_segment_idx = self._getProjectionIdx(current_pose)
        steer_map = [0, 0.5, 0.5, 0]
        velocity_map = [1.0, 1.0, 1.0, 1.0]
        return (
            velocity_map[projection_segment_idx],
            steer_map[projection_segment_idx],
            projection_segment_idx,
        )

from shapely.geometry import Point
def calculateCurvature(p1, p2, p3):
    # Create shapely points
    point1 = Point(p1)
    point2 = Point(p2)
    point3 = Point(p3)

    # Calculate vectors
    v1 = np.array([point2.x - point1.x, point2.y - point1.y])
    v2 = np.array([point3.x - point2.x, point3.y - point2.y])

    # Calculate angles between vectors
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    
    # Handle division by zero
    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0
    
    # Calculate angle in radians
    cos_theta = dot_product / (norm_v1 * norm_v2)
    
    # Clip the cosine value to avoid numerical issues
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    angle = np.arccos(cos_theta)

    # Assuming curvature k = angle / chord_length
    chord_length = norm_v1 + norm_v2
    curvature = angle / chord_length if chord_length != 0 else 0
    
    return curvature

def calcKappas(poses):
    if len(poses) < 3:
        print("invalid arguments")
        return 0.0
    
    return [calculateCurvature([a.x, a.y], [b.x, b.y], [c.x, c.y]) for (a, b, c) in zip(poses[0:-2], poses[1:-1], poses[2:])]

class UShapedTurnModel(UShapedTurnModelInterface):
    def __init__(self, drive_path: DrivePathInterface):
        self.drive_path = drive_path

    def _getProjectionIdx(self, pose):
        drive_path_poses = self.drive_path.getPoses()
        drive_path_segments = [EasyDict(start=a,end=b) for (a,b) in zip(drive_path_poses[0:-1], drive_path_poses[1:])]

        # find the nearest segment and take its index as result
        sumDistFunc = lambda current, segment: math.hypot(
            current.x - segment.start.x, current.y - segment.start.y
        ) + math.hypot(current.x - segment.end.x, current.y - segment.end.y)
        
        idx = 0
        sum_dist = 1e6
        for i in range(len(drive_path_segments)):
            tmp_sum_dist = sumDistFunc(pose, drive_path_segments[i])
            if tmp_sum_dist <= sum_dist:
                # print(i, tmp_sum_dist)
                idx = i
                sum_dist = tmp_sum_dist
        
        return idx

    def decode(self, current_pose):
        dp_poses = self.drive_path.getPoses()
        # steerFunc = lambda yaw: 0.5 if abs(yaw) > 0.001 else 0
        
        # # yaw is not currect
        # yaws = [math.atan2(b.y - a.y, b.x - a.x) for (a, b) in zip(dp_poses[0:-1], dp_poses[1:])]
        # for pose, yaw in zip(dp_poses[1:] ,yaws):
        #     pose.yaw = yaw

        kMaxSteer = 0.5
        kUTurnKappaThr = 0.02 # equal to radius=50
        kappas = calcKappas(dp_poses)
        steer_map = [(kMaxSteer if abs(kappa) > kUTurnKappaThr else 0.0) for kappa in kappas]
        steer_map = [steer_map[0]] + steer_map
        steer_map = steer_map + [steer_map[-1]]
        velocity_map = np.ones(len(dp_poses))

        projection_segment_idx = self._getProjectionIdx(current_pose)
        return (
            velocity_map[projection_segment_idx],
            steer_map[projection_segment_idx],
            projection_segment_idx,
        )

class PathDriver:
    def __init__(self, model: UShapedTurnModelFake):
        self.model = model
    
    def drive(self, pose):
        return self.model.decode(pose)

def testDrivePathFake():
    dp = DrivePathFake()
    model = UShapedTurnModelFake(dp)
    pd = PathDriver(model)
    print(
        "expected return is (1.0, 0.0, 0), return is: {}".format(
            pd.drive(EasyDict(x=0.0, y=-1.0, yaw=0))
        )
    )
    print(
        "expected return is (1.0, 0.5, 1), return is: {}".format(
            pd.drive(EasyDict(x=0.0, y=1.0, yaw=0))
        )
    )

def testUShapedTurnModelReal():
    poses_data = dict(xs=[0.0, 1.0, 2.0, 2.0], ys=[0.0, 0.0, 1.0, 2.0], yaws=[0.0, np.pi/4, np.pi/2, np.pi/2])
    poses = [EasyDict(x=a, y=b, yaw=c) for (a, b, c) in zip(poses_data['xs'], poses_data['ys'], poses_data['yaws'])]
    # print(poses)
    dp = DrivePath(poses)
    model = UShapedTurnModel(dp)
    
    samples_data = EasyDict(xs=[0.5, 1.5, 2.0], ys=[0.0, 0.5, 1.5], yaws=[0.0, 0.0, 0.0], truths=[0, 1, 2])
    samples = [EasyDict(x=a, y=b, yaw=c, truth=d) for (a, b, c, d) in zip(samples_data.xs, samples_data.ys, samples_data.yaws, samples_data.truths)]
    for sample in samples:
        idx = model._getProjectionIdx(sample)
        print("y={}, truth={}, decode={}".format(idx, sample.truth, model.decode(sample)))

if __name__ == "__main__":
    testDrivePathFake()
    testUShapedTurnModelReal()