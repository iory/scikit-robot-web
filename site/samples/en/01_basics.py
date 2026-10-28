# The basics with Jedy, JSK's small dual-arm mobile robot.
#
#   robot : the robot model you plan with. Changing it moves nothing.
#   ri    : the robot interface. It sends commands to the robot and reads
#           back what it did. Here the robot is simulated in MuJoCo; on a
#           real Jedy, ri is its ROS interface and the rest stays the same.
#
# In the 3D view, the translucent blue robot is `robot` and the solid one
# is what `ri` drives.
import numpy as np

import skrobot
from skrobot.interfaces import MujocoRobotInterface


# Wheel joint -> (radius [m], roller angle [rad]), so the wheels turn while
# the base drives. Jedy has mecanum wheels (rollers at 45 degrees), which
# let it move sideways as well.
WHEELS = {
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}

# light=True: simplified meshes that load quickly; the joints are the same.
robot = skrobot.models.Jedy(light=True)
robot.reset_pose()
ri = MujocoRobotInterface(
    robot, planar_base=True, wheels=WHEELS,
    base_max_velocity=(0.15, 1.0),  # a small robot drives slowly [m/s, rad/s]
    viewer=True)

# 1. Joint angles: change the model, then send its angle vector.
print('1. raise the right hand and look to the left')
robot.rarm_joint0.joint_angle(-0.5)
robot.head_joint0.joint_angle(np.deg2rad(30))
ri.angle_vector(robot.angle_vector(), 2.0)  # reach it in 2 seconds
ri.wait_interpolation()                     # block until the motion ends

# 2. Read back where the robot actually is.
actual = ri.angle_vector()
print('   rarm_joint0: sent {:.3f} rad, reached {:.3f} rad'.format(
    robot.rarm_joint0.joint_angle(),
    actual[robot.joint_list.index(robot.rarm_joint0)]))

# 3. Drive the base, in its own frame: x forward, y left, yaw counterclockwise.
print('2. drive 0.2 m forward, then turn 90 degrees')
ri.go_pos(0.2, 0.0, 0.0)
ri.go_pos(0.0, 0.0, np.pi / 2)
print('   odometry: x {:.3f} m, y {:.3f} m'.format(*ri.odom.worldpos()[:2]))

# 4. ri moved the robot, not the model: bring the model along.
robot.newcoords(ri.odom)
robot.angle_vector(ri.angle_vector())

# 5. Back to the starting pose.
print('3. back to the reset pose')
robot.reset_pose()
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()
print('done')
