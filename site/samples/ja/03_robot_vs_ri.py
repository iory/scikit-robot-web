# robot と ri の違い。
#   robot (青い半透明) : 頭の中の模型。書き換えると一瞬で変わり、机もすり抜ける。
#   ri    (ふつうの色) : 本物 (ここでは MuJoCo) への窓口。時間をかけて動き、
#                         重力や衝突で思いどおりにならないこともある。
# 「robot で考えて、ri で動かして、ri で確かめる」
import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.interfaces import MujocoRobotInterface
from skrobot.model import Box


# Jedy の手は低い所まで届かないので、机は高さ 10 cm にする。
TABLE_HEIGHT = 0.10
HAND_X, HAND_Y = 0.16, -0.08  # 右手を伸ばす場所 (ロボットの前)
FINGERS = ['rarm_link7', 'rarm_link8', 'larm_link7', 'larm_link8']  # 02_pick

robot = skrobot.models.Jedy(light=True)
robot.reset_pose()
# 机は半透明にして、中に入る青い模型の手を見えるようにする
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
    """robot (模型) の右手先を、前向きのまま高さ z へ。"""
    ok = robot.inverse_kinematics(
        Coordinates().translate((HAND_X, HAND_Y, z), 'world'),
        link_list=robot.rarm.link_list,
        move_target=robot.rarm_end_coords, stop=300)
    if ok is False:
        raise RuntimeError('IK が解けませんでした')


# 1. robot だけを書き換える: 青い模型は変わるが、本物は動かない
solve_ik(TABLE_HEIGHT + 0.03)
print('1. robot を書き換えた: 手先の高さ {:.3f} m / ri はまだ動いていない'
      .format(hand_z(robot)))

# 2. ri に送ると、本物が 2 秒かけて模型に重なる
print('2. ri に送る')
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()

# 3. 机の中 (天板の 4 cm 下) に手を入れる姿勢を計画する。
#    模型は机をすり抜けるが、本物は天板で止まる。
print('3. 机の中に手を入れる姿勢を送る')
solve_ik(TABLE_HEIGHT - 0.04)
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()

# 4. ri から読み返すと、計画 (robot) と現実 (ri) の差がわかる
planned = robot.angle_vector().copy()
actual = ri.angle_vector()
print('   計画した手先の高さ {:.3f} m'.format(hand_z(robot)))
print('   関節角度の差の最大 {:.3f} rad'.format(np.abs(actual - planned).max()))
print('   机に触れているもの', ri.contacts('table'))

# 5. 読み返した値を robot に入れると、模型が本物に重なる
robot.angle_vector(actual)
print('5. robot に読み返した後の手先の高さ {:.3f} m (天板 {:.2f} m)'.format(
    hand_z(robot), TABLE_HEIGHT))
