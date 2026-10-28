# robot versus ri.
#   robot (translucent blue): the model in your head. Change it and it
#                             changes at once, passing through the table.
#   ri    (solid)           : the way to the real robot (here MuJoCo). It
#                             takes time to move, and gravity and contact
#                             may stop it short of what you asked.
# Plan with robot, move with ri, check with ri.
import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.interfaces import MujocoRobotInterface
from skrobot.model import Box


# Jedy's hand does not reach low, so the table is 10 cm high.
TABLE_HEIGHT = 0.10
HAND_X, HAND_Y = 0.16, -0.08  # where the right hand goes, ahead of the robot
FINGERS = ['rarm_link7', 'rarm_link8', 'larm_link7', 'larm_link8']  # 02_pick

robot = skrobot.models.Jedy(light=True)
robot.reset_pose()
# A see-through table, to show the blue hand going into it.
table = Box((0.2, 0.4, TABLE_HEIGHT), face_colors=(160, 110, 60, 140),
            name='table')
table.translate((0.22, 0.0, TABLE_HEIGHT / 2))
ri = MujocoRobotInterface(
    robot, objects=[table],
    converter_kwargs={'convex_decompose_links': FINGERS,
                      'coacd_quality': 'fine'},
    viewer=True)


def hand_z(model):
    return model.rarm_end_coords.worldpos()[2]


def solve_ik(z):
    """Put the model's right hand at height z, still pointing forward."""
    ok = robot.inverse_kinematics(
        Coordinates().translate((HAND_X, HAND_Y, z), 'world'),
        link_list=robot.rarm.link_list,
        move_target=robot.rarm_end_coords, stop=300)
    if ok is False:
        raise RuntimeError('IK found no solution')


# 1. Change robot only: the blue model moves, the robot does not.
solve_ik(TABLE_HEIGHT + 0.03)
print('1. changed robot: hand at {:.3f} m / ri has not moved'.format(
    hand_z(robot)))

# 2. Send it to ri: over 2 seconds the robot moves onto the model.
print('2. send it to ri')
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()

# 3. Plan a hand 4 cm inside the table. The model goes through the top;
#    the robot stops on it.
print('3. send a hand inside the table')
solve_ik(TABLE_HEIGHT - 0.04)
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()

# 4. Read ri back: the plan (robot) and what happened (ri) differ.
planned = robot.angle_vector().copy()
actual = ri.angle_vector()
print('   planned hand height {:.3f} m'.format(hand_z(robot)))
print('   largest joint difference {:.3f} rad'.format(
    np.abs(actual - planned).max()))
print('   touching the table:', ri.contacts('table'))

# 5. Put what ri read into robot: the model moves onto the robot.
robot.angle_vector(actual)
print('5. after reading ri back, hand at {:.3f} m (table top {:.2f} m)'
      .format(hand_z(robot), TABLE_HEIGHT))
