# Jedy in room 13BC, the exercise room, among its 48 desks.
# The room is loaded from a URDF like a robot, and passed to ri as an
# object: its walls, floor and desks are fixed in place, with the
# photographs they were made from as textures.
import numpy as np

import skrobot
from skrobot.interfaces import MujocoRobotInterface
from skrobot.models.urdf import RobotModelFromURDF


WHEELS = {  # wheel joint -> (radius [m], roller angle [rad]); see 01_basics
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}

# The room: x runs along it (26.8 m), y across it (9.6 m), the floor is at
# z = 0. light=True: textures small enough for a browser.
room = RobotModelFromURDF(urdf_file=skrobot.data.room13bc_urdfpath(light=True))

robot = skrobot.models.Jedy(light=True)
robot.reset_pose()
# Stand in the aisle between the two rows of desks, facing along the room.
robot.translate((9.5, 5.2, 0.0))

ri = MujocoRobotInterface(
    robot, planar_base=True, wheels=WHEELS, base_max_velocity=(0.3, 1.0),
    objects=[room], viewer=True)

# 1. Down the aisle.
print('1. drive 3 m down the aisle')
ri.go_pos(3.0, 0.0, 0.0)
print('   at x {:.2f} m, y {:.2f} m'.format(*ri.odom.worldpos()[:2]))

# 2. Turn to a desk and drive under it. Its feet are bars 6 cm high, and
# Jedy's wheels are 6 cm across: ri stops on them.
print('2. turn left and drive 2 m, under a desk')
ri.go_pos(0.0, 0.0, np.pi / 2)
goal = ri.odom.copy_worldcoords().translate((2.0, 0.0, 0.0))
ri.go_pos(2.0, 0.0, 0.0)
print('   stopped at y {:.2f} m instead of {:.2f} m'.format(
    ri.odom.worldpos()[1], goal.worldpos()[1]))
touching = {link.name: ri.contacts(link.name) for link in robot.link_list}
print('   touching:', {name: other for name, other in touching.items() if other})

# 3. The model goes wherever it is told: it does not know about the desk.
robot.newcoords(goal)
print('3. robot (blue) where ri was asked to go, ri where it stopped')
