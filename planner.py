import os, sys

sys.path.append("./")
# add the path to the pybind11 module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "./build/")))

from vehicle_model import VehicleModel
from easydict import EasyDict
import numpy as np
import math
import copy


def isYawReached(current, target):
    # # return if parallel
    # return abs(math.cos(a) - math.cos(b)) < 1e-2 and abs(math.sin(a) - math.sin(b)) < 1e-2

    # return if nearly codirection
    return (
        math.cos(current) * math.cos(target) + math.sin(current) * math.sin(target)
        > 0
    )

from abc import ABC, abstractmethod

class PlannerInterface(ABC):
    @abstractmethod
    def plan(self, start_pose, end_pose, curvature=0.2, step_size=0.1):
        pass

from dubins_path_planner import plan_dubins_path
def Dubins(start_pose, end_pose, curvature, step_size=0.1, selected_types=None):
    start_x = start_pose.x  # [m]
    start_y = start_pose.y  # [m]
    start_yaw = start_pose.yaw  # [rad]

    end_x = end_pose.x  # [m]
    end_y = end_pose.y  # [m]
    end_yaw = end_pose.yaw  # [rad]

    path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
        start_x,
        start_y,
        start_yaw,
        end_x,
        end_y,
        end_yaw,
        curvature,
        step_size,
        selected_types,
    )

    # drop the path with too long turning
    for m, l in zip(mode, lengths):
        if m is not "S" and l > np.pi / curvature:
            return []
    return [EasyDict(x=a, y=b, yaw=c) for (a, b, c) in zip(path_x, path_y, path_yaw)]
class DubinsPlanner(PlannerInterface):
    def __init__(self):
        pass
    
    def plan(self, start_pose, end_pose, curvature=0.2, step_size=0.1):
        return Dubins(start_pose, end_pose, curvature, step_size)

class MultiRadiusPlanner(PlannerInterface):
    def __init__(self, divisions: int, once_planner: PlannerInterface):
        self.divisions = divisions
        self.once_planner = once_planner
    
    def plan(self, start_pose, end_pose, curvature=0.2, step_size=0.1):
        for curv in np.arange(curvature / self.divisions, curvature, curvature / self.divisions):
            path = self.once_planner.plan(start_pose, end_pose, curv)
            if len(path) > 0:
                return path
        
        return []

from u_shaped_turn import planUShapedTurn
class UShapedTurnPlanner(PlannerInterface):
    def __init__(self):
        pass
    
    def plan(self, start_pose, end_pose, curvature=0.2, step_size=0.1):
        start_pose_values = np.array([start_pose.x, start_pose.y, start_pose.yaw])
        end_pose_values = np.array([end_pose.x, end_pose.y, end_pose.yaw])
        trace = planUShapedTurn(start_pose_values, end_pose_values)
        return [EasyDict(x=pt[0], y=pt[1], yaw=pt[2]) for pt in trace]

import os, sys
sys.path.append("./")
# add the path to the pybind11 module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "./build/")))
from u_shaped_turn import planHybridAStar

class hybridAstarPlanner(PlannerInterface):
    def __init__(self):
        pass
    
    def plan(self, start_pose, end_pose, curvature=0.2, step_size=0.1):
        width = max(start_pose.x, end_pose.x) + 1
        height = max(start_pose.y, end_pose.y) + 1
        nStart = np.array([start_pose.x, start_pose.y, start_pose.yaw, 0, 0])
        nGoal = np.array([end_pose.x, end_pose.y, end_pose.yaw, 0, 0])
        path = planHybridAStar(width, height, nStart, nGoal)
        return [EasyDict(x=pt[0], y=pt[1], yaw=pt[2]) for pt in path]
        

from model import UShapedTurnModelInterface, PathDriver
def Infer(vm: VehicleModel, u_shaped_turn_model: UShapedTurnModelInterface, end_point, kVelocity, kDt):
    path_driver = PathDriver(u_shaped_turn_model)
    
    kMaxTraceLength = 50
    trace_poses = [copy.copy(vm.pose)]
    trace_projection2drive_path_segment_idxs = [0]
    kMaxLoop = int(kMaxTraceLength / kDt)

    loop_i = 0
    while not isYawReached(vm.pose.yaw, end_point.yaw) and loop_i < kMaxLoop:
        loop_i += 1
        velocity, steer, projection_segment_idx = path_driver.drive(vm.pose)
        vm.drive(kVelocity, steer, kDt)

        trace_poses.append(copy.copy(vm.pose))
        trace_projection2drive_path_segment_idxs.append(projection_segment_idx)
    
    if len(trace_poses) == 0:
        trace_poses = [vm.pose]
        trace_projection2drive_path_segment_idxs = [-1]

    return trace_poses, trace_projection2drive_path_segment_idxs

class directionShotsPlanner(PlannerInterface):
    def __init__(self, shot_steps: np.array, once_planner: PlannerInterface):
        self.shot_steps = shot_steps
        self.once_planner = once_planner
    
    def plan(self, start_pose, end_pose, curvature=0.2, step_size=0.1):
        for step in self.shot_steps:
            end_point_shoted = copy.copy(end_pose)
            # extend end_point forward
            end_point_shoted.x += math.cos(end_pose.yaw) * step
            end_point_shoted.y += math.sin(end_pose.yaw) * step
            
            path = self.once_planner.plan(start_pose, end_point_shoted, curvature, step_size)
            if len(path) > 0:
                return path
        
        return []

def Plan(start_point, end_point, u_shaped_turn_model):
    kVelocity = 1.0
    kDt = 0.4
    # print("plan {}->{}".format(start_point, end_point))
    vm = VehicleModel()
    vm.initPose(start_point.x, start_point.y, start_point.yaw)

    trace_poses, trace_projection2drive_path_segment_idxs = Infer(
        vm, u_shaped_turn_model, end_point, kVelocity, kDt
    )

    post_start_pose = trace_poses[-1]
    post_end_pose = end_point
    
    # post_planner = DubinsPlanner()
    # post_planner = hybridAstarPlanner()
    # post_planner = directionShotsPlanner(np.arange(0.0, 20.0, 1.0), DubinsPlanner())
    # post_planner = MultiRadiusPlanner(5, DubinsPlanner())
    # post_planner = directionShotsPlanner(np.arange(0.0, 20.0, 2.0), MultiRadiusPlanner(3, DubinsPlanner()))
    post_planner = directionShotsPlanner(np.arange(0.0, 20.0, 2.0), MultiRadiusPlanner(3, UShapedTurnPlanner()))
    post_path = post_planner.plan(
        post_start_pose,
        post_end_pose,
        math.tan(vm.max_delta) / vm.wheel_base,
        (kVelocity * kDt) / 5.0,
    )  # 5.0 due to bug in dubins lib
    post_path_projection2drive_path_segment_idxs = [-1] * len(
        post_path
    )  # no projection is valid, so all fake as -1

    traj = trace_poses + post_path
    projection2drive_path_segment_idxs = (
        trace_projection2drive_path_segment_idxs
        + post_path_projection2drive_path_segment_idxs
    )
    for pose, projection2drive_path_segment_idx in zip(
        traj, projection2drive_path_segment_idxs
    ):
        pose.projection2drive_path_segment_idxs = projection2drive_path_segment_idx
    return traj

def basicExample():
    start_point = EasyDict(x=0.0, y=0.0, yaw=0.0)
    end_point = EasyDict(x=0.0, y=10.0, yaw=np.pi)
    dubins_path = Dubins(start_point, end_point, 3.0, 0.5)
    print(dubins_path[0], dubins_path[-1])

    from drive_path import UShapedTurnModelFake, DrivePathFake
    u_shaped_turn_model = UShapedTurnModelFake(DrivePathFake())
    trace_poses = Plan(start_point, end_point, u_shaped_turn_model)
    print([pose.yaw for pose in trace_poses])

if __name__ == "__main__":
    # basicExample()
    
    import sys
    from utils.file import parseInput
    
    uturn_input = parseInput(sys.argv[1])
    
    from model import UShapedTurnModel
    from drive_path import DrivePath
    
    u_shaped_turn_model = UShapedTurnModel(DrivePath(uturn_input.drive_path))
    # from drive_path import UShapedTurnModel, DrivePathFake
    # u_shaped_turn_model = UShapedTurnModel(DrivePathFake())
    
    trace_poses = Plan(uturn_input.start_point, uturn_input.go_points[0], u_shaped_turn_model)
    print([pose.yaw for pose in trace_poses])
    
