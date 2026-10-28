# Jedy (JSK の小型双腕移動ロボット) で基本の操作。
#
#   robot : 計画に使うロボットの模型。書き換えても何も動かない。
#   ri    : ロボットインタフェース。ロボットに指令を送り、実際の状態を読む。
#           ここではロボットを MuJoCo でシミュレーションしている。実機の
#           Jedy では ri が ROS のインタフェースになり、残りは同じまま。
#
# 3D ビューの青い半透明のロボットが robot、ふつうの色のほうが ri の動かす
# ロボット。
import numpy as np

import skrobot
from skrobot.interfaces import MujocoRobotInterface


# 台車が動くと車輪も回るように: 関節名 -> (半径 [m], ローラーの角度 [rad])。
# Jedy はメカナムホイール (ローラーが 45 度傾いた車輪) なので横にも進める。
WHEELS = {
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}

# light=True: 形を簡略化した軽いモデル。読み込みが速く、関節は同じ。
robot = skrobot.models.Jedy(light=True)
robot.reset_pose()
ri = MujocoRobotInterface(
    robot, planar_base=True, wheels=WHEELS,
    base_max_velocity=(0.15, 1.0),  # 小さいロボットなのでゆっくり [m/s, rad/s]
    viewer=True)

# 1. 関節角度: 模型を変えてから、その角度ベクトルを送る。
print('1. 右手を上げて左を向く')
robot.rarm_joint0.joint_angle(-0.5)
robot.head_joint0.joint_angle(np.deg2rad(30))
ri.angle_vector(robot.angle_vector(), 2.0)  # 2 秒かけて動かす
ri.wait_interpolation()                     # 動き終わるまで待つ

# 2. ロボットが実際にどこにいるかを読む。
actual = ri.angle_vector()
print('   rarm_joint0: 送った値 {:.3f} rad, 実際 {:.3f} rad'.format(
    robot.rarm_joint0.joint_angle(),
    actual[robot.joint_list.index(robot.rarm_joint0)]))

# 3. 台車を動かす。台車から見て x が前、y が左、yaw は左回り。
print('2. 0.2 m 前に進んでから 90 度回る')
ri.go_pos(0.2, 0.0, 0.0)
ri.go_pos(0.0, 0.0, np.pi / 2)
print('   オドメトリ: x {:.3f} m, y {:.3f} m'.format(*ri.odom.worldpos()[:2]))

# 4. ri が動かしたのはロボットで、模型ではない。模型も連れてくる。
robot.newcoords(ri.odom)
robot.angle_vector(ri.angle_vector())

# 5. 最初の姿勢に戻す。
print('3. reset_pose に戻す')
robot.reset_pose()
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()
print('おわり')
