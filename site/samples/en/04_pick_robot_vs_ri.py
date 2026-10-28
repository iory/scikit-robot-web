# robot versus ri, through a pick (translucent blue = robot).
#   A. IK moves the blue model first; sent to ri, the robot catches up.
#   B. ri.go_pos drives the robot; robot stays where it was.
#   C. The fingers stop on the cube; robot thinks they closed all the way.
#   D. Ask ri where the cube is: the Box placed in the scene does not move.
import copy

import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.interfaces import MujocoRobotInterface
from skrobot.model import Box


TABLE_HEIGHT = 0.065
CUBE = 0.035

WHEELS = {  # wheel joint -> (radius [m], roller angle [rad]); see 01_basics
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}
FINGERS = ['rarm_link7', 'rarm_link8', 'larm_link7', 'larm_link8']  # 02_pick

robot = skrobot.models.Jedy(light=True)
robot.reset_pose()

table = Box((0.2, 0.4, TABLE_HEIGHT), face_colors=(160, 110, 60), name='table')
table.translate((0.55, 0.0, TABLE_HEIGHT / 2))
cube = Box((CUBE, CUBE, CUBE), face_colors=(220, 30, 30), name='cube')
cube.translate((0.47, -0.08, TABLE_HEIGHT + CUBE / 2))

ri = MujocoRobotInterface(
    robot, planar_base=True, wheels=WHEELS, base_max_velocity=(0.15, 1.0),
    objects=[table], movable_objects=[cube],
    converter_kwargs={'convex_decompose_links': FINGERS,
                      'coacd_quality': 'fine'},
    viewer=True)


def plan_arm(coords):
    """Move the model's right hand to coords. Nothing is sent to ri yet."""
    ok = robot.inverse_kinematics(
        coords, link_list=robot.rarm.link_list,
        move_target=robot.rarm_end_coords, stop=300)
    if ok is False:
        raise RuntimeError('IK found no solution')


def send(time=1.5):
    """Send the model's posture to ri and wait for the robot to get there."""
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()


def set_gripper(angle):
    """Right gripper, on the model only: 0 closes it, negative opens it."""
    robot.rarm_gripper_joint.joint_angle(angle)
    robot.rarm_mimic_gripper_joint.joint_angle(-angle)


real = copy.deepcopy(robot)  # a model to hold the robot's posture


def hand_error():
    """Distance between the right hands of robot and ri [m]."""
    real.newcoords(ri.odom)
    real.angle_vector(ri.angle_vector())
    return np.linalg.norm(real.rarm_end_coords.worldpos()
                          - robot.rarm_end_coords.worldpos())


side = Coordinates()  # the hand points forward

# A. The plan (robot) comes first, the motion (ri) after.
print('A. solve IK on robot (only the blue model moves)')
set_gripper(-1.0)
plan_arm(side.copy_worldcoords().translate((0.12, -0.08, 0.10), 'world'))
print('   hands {:.3f} m apart'.format(hand_error()))
print('   send it to ri')
send()
print('   hands {:.3f} m apart'.format(hand_error()))

# B. Only ri moves the base.
print('B. ri.go_pos(0.30, 0, 0)')
ri.go_pos(0.30, 0.0, 0.0)
print('   robot at x = {:.3f} m / ri.odom at x = {:.3f} m'.format(
    robot.worldpos()[0], ri.odom.worldpos()[0]))
print('   IK now would plan to reach from 0.30 m behind the robot')
robot.newcoords(ri.odom)
print('   robot.newcoords(ri.odom) brings the blue model to the robot')

# Slide the hand in from behind the cube; ask ri (a sensor) where it is.
p = ri.object_coords('cube').worldpos()
grasp = p + [0.0125, 0.0, 0.008]
plan_arm(side.copy_worldcoords().translate(grasp + [-0.05, 0, 0], 'world'))
send()
plan_arm(side.copy_worldcoords().translate(grasp, 'world'))
send()

# C. Close the fingers: robot thinks they reached 0.3 rad; the cube stops them.
print('C. close the gripper')
set_gripper(0.3)
send(1.0)
g = robot.joint_list.index(robot.rarm_gripper_joint)
print('   fingers: robot {:.3f} rad / ri {:.3f} rad (stopped by the cube)'
      .format(robot.angle_vector()[g], ri.angle_vector()[g]))
print('   touching the cube:', ri.contacts('cube'))

plan_arm(side.copy_worldcoords().translate(grasp + [0, 0, 0.04], 'world'))
send()

# D. The Box in the scene stays where it was placed; ri knows where it is.
print('D. cube height: cube.worldpos() {:.3f} m / ri.object_coords {:.3f} m'
      .format(cube.worldpos()[2], ri.object_coords('cube').worldpos()[2]))
rise = ri.object_coords('cube').worldpos()[2] - p[2]
print('success!' if rise > 0.02 else 'failed...')
