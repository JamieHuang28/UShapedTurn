import numpy as np


from bokeh.models import ColumnDataSource

from bokeh.models import Arrow, NormalHead, OpenHead, VeeHead
from bokeh.palettes import Spectral7

import math

class LineDataSource:
    def __init__(self):
        self.xs = np.arange(3)
        self.ys = np.arange(3)
        self.source = ColumnDataSource(self.get())
    
    def get(self):
        return dict(xs=self.xs, ys=self.ys)
    
    def updateData(self, points):
        self.xs = [point.x for point in points]
        self.ys = [point.y for point in points]
        self.source.data = self.get()
    
    def registerRender(self, plot):
        plot.line(
            "xs", "ys", source=self.source, line_width=3, line_alpha=0.6, color="#0000ff"
        )

class RoadCurbDataSource:
    def __init__(self, defualt_lane_width):
        self.x = np.array([-defualt_lane_width, -defualt_lane_width])
        self.y = np.array([-10.0, 10.0])
        self.source = ColumnDataSource(data=self.get())

    def updateData(self, x):
        self.x = np.array([x, x])
        self.source.data = self.get()

    def get(self):
        return dict(x=self.x, y=self.y)

    def registerRender(self, plot):
        plot.line(
            "x", "y", source=self.source, line_width=3, line_alpha=0.6, color="#0000ff"
        )


class PoseDataSource:
    def __init__(self, default_dict):
        self.x = default_dict.x
        self.y = default_dict.y
        self.yaw = default_dict.yaw
        try:
            self.color = default_dict.color
        except:
            self.color = "#000000"
        self.source = ColumnDataSource(data=self.get())
        
    def updateData(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.yaw = pose.yaw
        self.source.data = self.get()

    def get(self):
        return dict(
            start_x=[self.x],
            start_y=[self.y],
            end_x=[self.x + math.cos(self.yaw)],
            end_y=[self.y + math.sin(self.yaw)],
        )

    def registerRender(self, p):
        p.add_layout(
            Arrow(
                end=NormalHead(size=10, fill_color=self.color, line_color=self.color),
                source=self.source,
                x_start="start_x",
                y_start="start_y",
                x_end="end_x",
                y_end="end_y",
            )
        )


from bokeh.models import Dot


class TurnAnchorDataSource:
    def __init__(self):
        self.x = [0.0]
        self.y = [0.0]
        self.sizes = [20.0]
        self.source = ColumnDataSource(self.get())

    def updateData(self, x, y):
        self.x = np.array([0.0]) + x
        self.y = np.array([0.0]) + y
        self.source.data = self.get()

    def get(self):
        return dict(x=self.x, y=self.y, sizes=self.sizes)

    def registerRender(self, plot):
        glyph = Dot(x="x", y="y", size="sizes", line_color="#dd1c77", fill_color=None)
        plot.add_glyph(self.source, glyph)


class MultiLineDataSource:
    def __init__(self, data):
        self.data = data
        self.source = ColumnDataSource(self.get())
    
    def get(self):
        start_end_xs = [(segment.start.x, segment.end.x) for segment in self.data]
        start_end_ys = [(segment.start.y, segment.end.y) for segment in self.data]
        colors = [Spectral7[::-1][i%len(Spectral7)] for i in range(len(self.data))]
        return dict(segment_xs=start_end_xs, segment_ys=start_end_ys, colors=colors)
    
    def updateData(self, data):
        self.data = data
        self.source.data = self.get()
    
    def registerRender(self, p):
        line_opts = dict(
            line_width=2,
            line_color="colors",
            line_alpha=0.9,
            hover_line_color="colors",
            hover_line_alpha=1.0,
        )
        p.multi_line(source=self.source, xs="segment_xs", ys="segment_ys", **line_opts)


from drive_path import DrivePathInterface


class DrivePathDataSource:
    def __init__(self, drive_path: DrivePathInterface):
        self.drive_path = drive_path
        self.source = ColumnDataSource(self.get())

    def get(self):
        # return self.drive_path.getPointsAndArrow()

        return self.drive_path.getSegments()

    def updateData(self):
        # no update is required
        pass

    def registerRender(self, p):
        # p.add_layout(Arrow(end=OpenHead(size=3), source=self.source, x_start='x_start', y_start='y_start', x_end='x_end_arrow', y_end='y_end_arrow'))
        # p.line(source=self.source, x='x_start', y='y_start', line_width=3, line_color="#ffff00")

        line_opts = dict(
            line_width=5,
            line_color="colors",
            line_alpha=0.9,
            hover_line_color="colors",
            hover_line_alpha=1.0,
        )
        p.multi_line(source=self.source, xs="segment_xs", ys="segment_ys", **line_opts)


class TrajDataSource:
    def __init__(self):
        self.x = [0.0, 0.0, 0.0]
        self.y = [0.0, 1.0, 2.0]
        self.colors = ["ff0000", "00ff00", "#0000ff"]
        self.source = ColumnDataSource(self.get())

    def get(self):
        return dict(
            x=self.x, y=self.y, sizes=np.ones(len(self.x)) * 5, colors=self.colors
        )

    def updateData(self, traj):
        self.x = [pose.x for pose in traj]
        self.y = [pose.y for pose in traj]
        self.colors = [
            Spectral7[::-1][pose.projection2drive_path_segment_idxs] for pose in traj
        ]
        self.source.data = self.get()

    def registerRender(self, plot):
        plot.circle(
            source=self.source,
            x="x",
            y="y",
            size="sizes",
            color="peachpuff",
            fill_color="colors",
            alpha=0.7,
        )
