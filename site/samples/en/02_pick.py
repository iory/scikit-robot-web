# Pick a cube off a low table with Jedy, gripping it from the side.
# Jedy is about 30 cm tall, so the table and the cube are small too.
import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.interfaces import MujocoRobotInterface
from skrobot.model import Box


TABLE_HEIGHT = 0.065
CUBE = 0.035  # fits between the closed fingers' bars (about 3.8 cm apart)

WHEELS = {  # wheel joint -> (radius [m], roller angle [rad]); see 01_basics
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}
# The fingers are gears and bars, a hollow shape. MuJoCo collides meshes by
# their convex hull, which would fill the space between the fingers, so
# these links are split into convex parts (precomputed for the browser).
FINGERS = ['rarm_link7', 'rarm_link8', 'larm_link7', 'larm_link8']

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


def move_arm(coords, time=1.5):
    """Move the right hand to coords; stop if IK finds no solution."""
    ok = robot.inverse_kinematics(
        coords,
        link_list=robot.rarm.link_list,
        move_target=robot.rarm_end_coords,
        stop=300)  # at most this many iterations
    if ok is False:
        raise RuntimeError('IK found no solution')
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()


def gripper(angle, time=1.0):
    """Right gripper: 0 closes it, negative opens it (both fingers move)."""
    robot.rarm_gripper_joint.joint_angle(angle)
    robot.rarm_mimic_gripper_joint.joint_angle(-angle)
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()


# The hand points forward (x ahead); the fingers close from left and right.
side = Coordinates()

# 1. Open the hand and hold it a little above the table.
gripper(-1.0)
move_arm(side.copy_worldcoords().translate((0.12, -0.08, 0.10), 'world'))

# 2. Drive up to the table.
ri.go_pos(0.30, 0.0, 0.0)
# ri drove the simulated robot only. Move the model to where it now stands,
# or IK would reach out from where it started.
robot.newcoords(ri.odom)

# 3. Slide the hand in from behind the cube, squeeze, lift.
# The hand's tip goes a little past the cube's centre, so the cube sits in
# the middle of the fingers' bars, and a little above it: the fingers are
# 4 cm tall, and at the centre's height their lower edge hits the table.
p = ri.object_coords('cube').worldpos()  # where the cube is now, from ri
grasp = p + [0.0125, 0.0, 0.008]
move_arm(side.copy_worldcoords().translate(grasp + [-0.05, 0, 0], 'world'))
move_arm(side.copy_worldcoords().translate(grasp, 'world'))
gripper(0.3)
print('touching the cube:', ri.contacts('cube'))
move_arm(side.copy_worldcoords().translate(grasp + [0, 0, 0.04], 'world'))

rise = ri.object_coords('cube').worldpos()[2] - p[2]
print('lifted by {:.3f} m'.format(rise))
print('success!' if rise > 0.02 else 'failed...')
