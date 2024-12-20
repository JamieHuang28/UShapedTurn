from vehicle_model import VehicleModel
from easydict import EasyDict
import numpy as np
import math
import copy


def isYawReached(current, target):
    # # return if parallel
    # return abs(math.cos(a) - math.cos(b)) < 1e-2 and abs(math.sin(a) - math.sin(b)) < 1e-2

    # return if perpendicular
    return (
        abs(math.cos(current) * math.cos(target) + math.sin(current) * math.sin(target))
        < 0.1
    )


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


from drive_path import DrivePath


def InferByQuery(vm: VehicleModel, drive_path: DrivePath, end_point, kVelocity, kDt):
    kMaxTraceLength = 50
    trace_poses = []
    trace_projection2drive_path_segment_idxs = []
    kMaxLoop = int(kMaxTraceLength / kDt)

    loop_i = 0
    while not isYawReached(vm.pose.yaw, end_point.yaw) and loop_i < kMaxLoop:
        loop_i += 1
        velocity, steer, projection_segment_idx = drive_path.Query(vm.pose)
        vm.drive(kVelocity, steer, kDt)

        trace_poses.append(copy.copy(vm.pose))
        trace_projection2drive_path_segment_idxs.append(projection_segment_idx)
    
    if len(trace_poses) == 0:
        trace_poses = [vm.pose]
        trace_projection2drive_path_segment_idxs = [-1]

    return trace_poses, trace_projection2drive_path_segment_idxs


def Plan(start_point, end_point, drive_path):
    kVelocity = 1.0
    kDt = 0.4
    # print("plan {}->{}".format(start_point, end_point))
    vm = VehicleModel()
    vm.initPose(start_point.x, start_point.y, start_point.yaw)

    trace_poses, trace_projection2drive_path_segment_idxs = InferByQuery(
        vm, drive_path, end_point, kVelocity, kDt
    )

    dubins_start_pose = trace_poses[-1]
    dubins_end_pose = end_point
    dubins_path = Dubins(
        dubins_start_pose,
        dubins_end_pose,
        math.tan(vm.max_delta) / vm.wheel_base,
        (kVelocity * kDt) / 5.0,
    )  # 5.0 due to bug in dubins lib
    dubins_path_projection2drive_path_segment_idxs = [-1] * len(
        dubins_path
    )  # no projection is valid, so all fake as -1

    traj = trace_poses + dubins_path
    projection2drive_path_segment_idxs = (
        trace_projection2drive_path_segment_idxs
        + dubins_path_projection2drive_path_segment_idxs
    )
    for pose, projection2drive_path_segment_idx in zip(
        traj, projection2drive_path_segment_idxs
    ):
        pose.projection2drive_path_segment_idxs = projection2drive_path_segment_idx
    return traj


if __name__ == "__main__":
    start_point = EasyDict(x=0.0, y=0.0, yaw=0.0)
    end_point = EasyDict(x=0.0, y=10.0, yaw=np.pi)
    dubins_path = Dubins(start_point, end_point, 3.0, 0.5)
    print(dubins_path[0], dubins_path[-1])

    drive_path = DrivePath()
    trace_poses = Plan(start_point, end_point, drive_path)
    print([pose.yaw for pose in trace_poses])
