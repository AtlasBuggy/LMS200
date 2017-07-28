from atlasbuggy.logparser import LogParser
from atlasbuggy import Robot
from atlasbuggy.subscriptions import *

from mylms import MyLMS


class Simulator(LogParser):
    def __init__(self, file_name, directory="", enabled=True, log_level=None):
        super(Simulator, self).__init__(file_name, directory, enabled, log_level=log_level, update_rate=0.01)

        self.lms_parser = None
        self.lms_parser_tag = "parser"

    def take_from_log(self, subscriptions):
        self.lms_parser = subscriptions[self.lms_parser_tag].get_stream()
        self.lms_parser.initialized()

    def stop(self):
        self.lms_parser.stop()

    def stopped(self):
        self.lms_parser.stopped()

robot = Robot(log_level=10, write=False)

lms200 = MyLMS(make_image=False)
simulator = Simulator("logs/2017_Jun_15/11;00;13.log.xz")

simulator.subscribe(Subscription(simulator.lms_parser_tag, lms200))

robot.run(simulator, lms200.plotter)
