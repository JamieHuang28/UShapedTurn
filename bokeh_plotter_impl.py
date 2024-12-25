from easydict import EasyDict

from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.plotting import figure
from bokeh.models import Slider

from drawer_plotter_abstract import PlotterImplInterface
import data_sources

class BokehPlotter(PlotterImplInterface):
    def __init__(self):
        # Set up plot
        self.plot = figure(
            height=400,
            width=400,
            title="uturn scene",
            tools="crosshair,pan,reset,save,wheel_zoom",
            y_range=[-25.0, 25.0],
            x_range=[-25.0, 25.0],
        )
    
    def plotLine(self, line):
        line_data_source = data_sources.LineDataSource()
        line_data_source.updateData(line)
        line_data_source.registerRender(self.plot)
    
    def plotPose(self, pose):
        pose_data_source = data_sources.PoseDataSource(pose)
        pose_data_source.registerRender(self.plot)
    
    def plotMultiLines(self, lines):
        # line_data_sources = []
        # for line_segment in lines:
        #     ds = data_sources.LineDataSource()
        #     ds.updateData([line_segment.start, line_segment.end])
        #     line_data_sources.append(ds)
        
        # for ds in line_data_sources:
        #     ds.registerRender(self.plot)
        
        muti_line_data_source = data_sources.MultiLineDataSource(lines)
        muti_line_data_source.registerRender(self.plot)
    
    def show(self):
        curdoc().add_root(row(self.plot, width=800))
        curdoc().title = "UShapedTurn"