from mylms import MyLMS
from atlasbuggy.robot import Robot

log = True

robot = Robot(write=log)
lms200 = MyLMS(True, make_image=log)
robot.run(lms200, lms200.plotter)
