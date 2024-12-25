import os, sys

sys.path.append("./")
# add the path to the pybind11 module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "./build/")))

import numpy as np
from easydict import EasyDict

from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.plotting import figure
from bokeh.models import Slider

from data_sources import (
    RoadCurbDataSource,
    PoseDataSource,
    TurnAnchorDataSource,
    DrivePathDataSource,
    TrajDataSource,
)

from planner import Plan
from drive_path import DrivePathFake, UShapedTurnModel
from bokeh.palettes import Spectral7

# Set up data
drive_path = DrivePathFake()
u_shaped_turn_model = UShapedTurnModel(drive_path)

kDefaultTargetLaneWidth = 10.0
road_curb_data_source = RoadCurbDataSource(kDefaultTargetLaneWidth)

# kDefaultStartPoint = EasyDict(dict(x=6.0, y=-2.0, yaw=np.pi / 2, color=Spectral7[6]))
kDefaultStartPoint = drive_path.getPoses()[0]
kDefaultStartPoint.color = Spectral7[6]
start_pose_data_source = PoseDataSource(kDefaultStartPoint)

kDefaultEndPoint = EasyDict(dict(x=-6.0, y=-4.0, yaw=-np.pi / 2, color=Spectral7[0]))
end_pose_data_source = PoseDataSource(kDefaultEndPoint)

turn_anchor_data_source = TurnAnchorDataSource()

drive_path_data_source = DrivePathDataSource(drive_path)
traj_data_source = TrajDataSource()

# Set up plot
plot = figure(
    height=400,
    width=400,
    title="uturn scene",
    tools="crosshair,pan,reset,save,wheel_zoom",
    y_range=[-25.0, 25.0],
    x_range=[-25.0, 25.0],
)

road_curb_data_source.registerRender(plot)
start_pose_data_source.registerRender(plot)
end_pose_data_source.registerRender(plot)
turn_anchor_data_source.registerRender(plot)
drive_path_data_source.registerRender(plot)
traj_data_source.registerRender(plot)

# set up widgets
target_lane_road_curb = Slider(
    title="target_lane_edge",
    value=-kDefaultTargetLaneWidth,
    start=-20.0,
    end=0.0,
    step=0.2,
)

start_point_idx = Slider(
    title="start_point_idx", value=0, start=0, end=max(0, drive_path.numPoints()-1), step=int(1)
)
start_point_yaw_offset = Slider(
    title="start_point_yaw_offset", value=0.0, start=-np.pi, end=np.pi, step=0.1
)
start_point_lon_offset = Slider(
    title="start_point_lon_offset", value=0.0, start=-5.0, end=5.0, step=0.1
)
start_point_lat_offset = Slider(
    title="start_point_lat_offset", value=0.0, start=-2.0, end=2.0, step=0.1
)

end_point_x = Slider(
    title="end_point_x", value=kDefaultEndPoint.x, start=-20.0, end=0.0, step=0.2
)
end_point_y = Slider(
    title="end_point_y", value=kDefaultEndPoint.y, start=-20.0, end=20.0, step=0.2
)
end_point_yaw = Slider(
    title="end_point_yaw", value=kDefaultEndPoint.yaw, start=-np.pi, end=np.pi, step=0.1
)

import math
import copy


def update_data(attrname, old, new):
    def addOffset(
        pose_ref,
        yaw_offset_widget,
        lon_offset_widget,
        lat_offset_widget,
    ):
        # start_pose = copy.copy(start_pose_ref)
        # start_pose.yaw = start_pose_ref.yaw + start_point_yaw_offset.value
        # start_pose.x = start_pose_ref.x + start_point_lon_offset.value
        # start_pose.y = start_pose_ref.y + start_point_lat_offset.value

        pose = copy.copy(pose_ref)
        pose.yaw = pose_ref.yaw + yaw_offset_widget.value
        start_pose_x_offset = (
            math.cos(pose.yaw) * lon_offset_widget.value
            + -math.sin(pose.yaw) * lat_offset_widget.value
        )
        start_pose_y_offset = (
            math.sin(pose.yaw) * lon_offset_widget.value
            + math.cos(pose.yaw) * lat_offset_widget.value
        )
        pose.x = pose_ref.x + start_pose_x_offset
        pose.y = pose_ref.y + start_pose_y_offset

        return pose

    # Get the current slider values and update data source
    road_curb_data_source.updateData(target_lane_road_curb.value)

    # start_pose = EasyDict(
    #     x=start_point_x.value, y=start_point_y.value, yaw=start_point_yaw.value
    # )
    start_pose = addOffset(
        drive_path.getPoses()[start_point_idx.value],
        start_point_yaw_offset,
        start_point_lon_offset,
        start_point_lat_offset,
    )
    end_pose = EasyDict(
        x=end_point_x.value, y=end_point_y.value, yaw=end_point_yaw.value
    )
    start_pose_data_source.updateData(start_pose)
    end_pose_data_source.updateData(end_pose)
    traj = Plan(start_pose, end_pose, u_shaped_turn_model)
    traj_data_source.updateData(traj)


for w in [
    start_point_idx,
    start_point_yaw_offset,
    start_point_lon_offset,
    start_point_lat_offset,
    target_lane_road_curb,
    end_point_x,
    end_point_y,
    end_point_yaw,
]:
    w.on_change("value", update_data)

# Set up layouts and add to document
inputs = column(
    start_point_idx,
    start_point_yaw_offset,
    start_point_lon_offset,
    start_point_lat_offset,
    target_lane_road_curb,
    end_point_x,
    end_point_y,
    end_point_yaw,
)

curdoc().add_root(row(inputs, plot, width=800))
curdoc().title = "UShapedTurn"

# from bokeh.io import show
# show(plot)
