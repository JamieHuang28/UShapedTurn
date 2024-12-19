from vehicle_model import VehicleModel
from easydict import EasyDict
import numpy as np
import math
import copy

def matchYaw(a, b):
    # # return if parallel
    # return abs(math.cos(a) - math.cos(b)) < 1e-2 and abs(math.sin(a) - math.sin(b)) < 1e-2
    # return if perpendicular
    return abs(math.cos(a) * math.cos(b) + math.sin(a)*math.sin(b)) < 0.1

from dubins_path_planner import plan_dubins_path
def Dubins(start_pose, end_pose, curvature, step_size=0.1, selected_types=None):
    start_x = start_pose.x  # [m]
    start_y = start_pose.y  # [m]
    start_yaw = start_pose.yaw  # [rad]

    end_x = end_pose.x  # [m]
    end_y = end_pose.y  # [m]
    end_yaw = end_pose.yaw  # [rad]

    path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(start_x,
                                                               start_y,
                                                               start_yaw,
                                                               end_x,
                                                               end_y,
                                                               end_yaw,
                                                               curvature,
                                                               step_size,
                                                               selected_types)
    return [EasyDict(x=a, y=b, yaw=c) for (a, b, c) in zip(path_x, path_y, path_yaw)]
    
def Plan(start_point, end_point, drive_path):
    kWheelBase = 3.0
    kMaxSteer = 0.5
    kVelocity = 1.0
    kDt = 0.2
    kMaxTraceLength = 50
    # print("plan {}->{}".format(start_point, end_point))
    vm = VehicleModel(kWheelBase, kMaxSteer)
    vm.initPose(start_point.x, start_point.y, start_point.yaw)

    trace_poses = []
    loop_max = int(kMaxTraceLength / kDt)
    loop_i = 0
    while not matchYaw(vm.pose.yaw, end_point.yaw) and loop_i < loop_max:
        loop_i += 1
        velocity, steer = drive_path.Query(vm.pose)
        vm.drive(kVelocity, steer, kDt)
        trace_poses.append(copy.copy(vm.pose))
    
    dubins_start_pose = trace_poses[-1]
    dubins_end_pose = end_point
    dubins_path = Dubins(dubins_start_pose, dubins_end_pose, math.tan(kMaxSteer) / kWheelBase, kVelocity * kDt)

    return trace_poses + dubins_path

if __name__ == "__main__":
    start_point = EasyDict(x=0.0, y=0.0, yaw=0.0)
    end_point = EasyDict(x=0.0, y=10.0, yaw=np.pi)
    dubins_path = Dubins(start_point, end_point, 3.0, 0.5)
    print(dubins_path[0], dubins_path[-1])
    
    from drive_path import DrivePath
    drive_path = DrivePath()
    trace_poses = Plan(start_point, end_point, drive_path)
    print([pose.yaw for pose in trace_poses])

