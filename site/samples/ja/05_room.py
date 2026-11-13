# 演習室 13BC の Jedy。部屋には机が 48 台ある。
# 部屋はロボットと同じように URDF から読み込み、物体として ri に渡す。
# 壁・床・机はその場に固定され、作るのに使った写真がテクスチャとして
# 貼られている。
import numpy as np

import skrobot
from skrobot.interfaces import MujocoRobotInterface
from skrobot.models.urdf import RobotModelFromURDF


WHEELS = {  # 車輪の関節 -> (半径 [m], ローラの角度 [rad])。01_basics を参照
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}

# 部屋: x が長手 (26.8 m)、y が短手 (9.6 m)、床が z = 0。
# light=True: ブラウザで扱える大きさのテクスチャ。
room = RobotModelFromURDF(urdf_file=skrobot.data.room13bc_urdfpath(light=True))

robot = skrobot.models.Jedy(light=True)
robot.reset_pose()
# 机の 2 列のあいだの通路に、部屋の長手を向いて立つ。
robot.translate((9.5, 5.2, 0.0))

ri = MujocoRobotInterface(
    robot, planar_base=True, wheels=WHEELS, base_max_velocity=(0.3, 1.0),
    objects=[room], viewer=True)

# 1. 通路を進む。
print('1. 通路を 3 m 進む')
ri.go_pos(3.0, 0.0, 0.0)
print('   x {:.2f} m, y {:.2f} m にいる'.format(*ri.odom.worldpos()[:2]))

# 2. 机のほうを向いて、机の下に入る。机の脚は高さ 6 cm の棒で、Jedy の
# 車輪の直径も 6 cm: ri はそこで止まる。
print('2. 左を向いて 2 m 進み、机の下に入る')
ri.go_pos(0.0, 0.0, np.pi / 2)
goal = ri.odom.copy_worldcoords().translate((2.0, 0.0, 0.0))
ri.go_pos(2.0, 0.0, 0.0)
print('   y {:.2f} m まで行かず、y {:.2f} m で止まった'.format(
    goal.worldpos()[1], ri.odom.worldpos()[1]))
touching = {link.name: ri.contacts(link.name) for link in robot.link_list}
print('   触れているもの:', {name: other for name, other in touching.items() if other})

# 3. 模型は言われたところへ行く。机のことは知らない。
robot.newcoords(goal)
print('3. robot (青) は ri に頼んだ場所、ri は止まった場所')
