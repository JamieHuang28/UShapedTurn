import numpy as np


from bokeh.models import ColumnDataSource

from bokeh.models import Arrow, NormalHead, OpenHead, VeeHead

import math

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
        plot.line('x', 'y', source=self.source, line_width=3, line_alpha=0.6)

class PoseDataSource:
    def __init__(self, default_dict):
        self.x = default_dict.x
        self.y = default_dict.y
        self.yaw = default_dict.yaw
        self.source = ColumnDataSource(data=self.get())
    
    def updateData(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.source.data = self.get()
    
    def get(self):
        return dict(start_x=[self.x], start_y=[self.y], end_x=[self.x + math.cos(self.yaw)], end_y=[self.y+math.sin(self.yaw)])
    
    def registerRender(self, p):
        p.add_layout(Arrow(end=NormalHead(size=15), source=self.source, x_start='start_x', y_start='start_y', x_end='end_x', y_end='end_y'))


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


from easydict import EasyDict
class DrivePathDataSource:
    def __init__(self, drive_path):
        self.drive_path = drive_path
        self.source = ColumnDataSource(self.get())
    
    def get(self):
        xs, ys, yaws = self.drive_path.get()
        end_xs = [a + math.cos(b) for (a,b) in zip(xs, yaws)]
        end_ys = [a + math.sin(b) for (a,b) in zip(ys, yaws)]
        return dict(x_start=xs, y_start=ys, x_end=end_xs, y_end=end_ys)
    
    def updateData(self):
        # no update is required
        pass
    
    def registerRender(self, p):
        p.add_layout(Arrow(end=OpenHead(size=3), source=self.source, x_start='x_start', y_start='y_start', x_end='x_end', y_end='y_end'))
        p.line(source=self.source, x='x_start', y='y_start', line_width=3, line_color="#ffff00")

class TrajDataSource:
    def __init__(self):
        self.x = [0.0, 0.0, 0.0]
        self.y = [0.0, 1.0, 2.0]
        self.source = ColumnDataSource(self.get())
    
    def get(self):
        return dict(x=self.x, y=self.y, sizes=np.ones(len(self.x)) * 2)
    
    def updateData(self, poses):
        self.x = [pose.x for pose in poses]
        self.y = [pose.y for pose in poses]
        self.source.data = self.get()
    
    def registerRender(self, plot):
        plot.circle(source=self.source, x="x", y="y", size="sizes", color="navy", alpha=0.5)