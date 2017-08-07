import logging
import math
import time

import matplotlib.cm as colormap
import numpy as np

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from sicklms import SickLMS
from slam import SLAM


class MyLMS(SickLMS):
    def __init__(self, enabled=True, is_live=True, make_image=True):
        self.map_size_pixels = 1600
        self.map_size_meters = 50

        self.scan_plot = RobotPlot("lms200", marker='.', linestyle='')
        self.slam_plot = RobotPlot("slam")

        self.plotter = LivePlotter(2, self.scan_plot, self.slam_plot, enabled=True,
                                   default_resize_behavior=False,
                                   matplotlib_events=dict(key_press_event=self.key_press_fn),
                                   close_when_finished=True)

        self.plotter.get_axis(self.slam_plot).set_aspect("auto")
        self.plotter.get_axis(self.slam_plot).set_autoscale_on(True)
        self.plotter.get_axis(self.slam_plot).set_xlim([0, self.map_size_pixels])
        self.plotter.get_axis(self.slam_plot).set_ylim([0, self.map_size_pixels])

        self.is_live = is_live

        super(MyLMS, self).__init__("/dev/cu.usbserial", enabled=enabled, log_level=logging.INFO)

        if self.is_live:
            self.scan_size = 361
            self.detection_angle_degrees = 180
            self.scan_resolution = 0.5
            self.angles = np.arange(0, math.radians(self.detection_angle_degrees + self.scan_resolution),
                                    math.radians(self.scan_resolution))
        self.slam = None
        self.make_image = make_image

        self.prev_t = 0.0

    def start_up_commands(self):
        self.set_range(16)

    def initialized(self):
        self.slam = SLAM(self.map_size_pixels, self.map_size_meters, self.scan_size, 5, self.detection_angle_degrees,
                         self.scan_resolution)

    def point_cloud_received(self, point_cloud):
        self.scan_plot.update(point_cloud[:, 0], point_cloud[:, 1])

        if self.slam is not None:
            distances = self.distances * 1000
            self.slam.update(distances.tolist(), [0, 0, self.dt() - self.prev_t])

            map_img = np.reshape(np.frombuffer(self.slam.mapbytes, dtype=np.uint8),
                                 (self.map_size_pixels, self.map_size_pixels))

            self.plotter.draw_image(self.slam_plot, map_img, cmap=colormap.gray)
            self.prev_t = self.dt()

    def key_press_fn(self, event):
        if event.key == "q":
            self.plotter.exit()

    def time_started(self):
        if self.is_live:
            return time.time()
        else:
            return None

    def receive_log(self, log_level, message, line_info):
        if self.start_time is None:
            self.start_time = line_info["timestamp"]
        else:
            self.timestamp = line_info["timestamp"]

        if message.startswith(self.distance_log_tag):
            self.distances = np.array([float(item) for item in message[len(self.distance_log_tag) + 1:].split(" ")])
            self.point_cloud = np.vstack(
                [self.distances * np.cos(self.angles), self.distances * np.sin(self.angles)]).T
            self.point_cloud_received(self.point_cloud)

    def stopped(self):
        if self.slam is not None and self.make_image:
            self.slam.make_image("maps/" + self._log_info["file_name"] + " map")
