# ピックで見る robot と ri の違い (青い半透明 = robot)。
#   A. robot で IK を解くと青い模型が先に動き、ri に送ると本物が追いつく
#   B. ri.go_pos で本物は進むが、robot は元の場所に残る
#   C. 指は箱で止まる。robot は閉じきったつもりでいる
#   D. 箱の位置は ri から読む。シーンに置いた Box は動かない
import copy

import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.interfaces import MujocoRobotInterface
from skrobot.model import Box


TABLE_HEIGHT = 0.065
CUBE = 0.035

WHEELS = {  # 関節名 -> (半径 [m], ローラーの角度 [rad])。01_basics を参照
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
    """robot (模型) の右手先を coords へ。ri にはまだ送らない。"""
    ok = robot.inverse_kinematics(
        coords, link_list=robot.rarm.link_list,
        move_target=robot.rarm_end_coords, stop=300)
    if ok is False:
        raise RuntimeError('IK が解けませんでした')


def send(time=1.5):
    """robot の姿勢を ri に送って、本物が追いつくまで待つ。"""
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()


def set_gripper(angle):
    """右グリッパー (robot だけ): 0 で閉じ、負にすると開く。"""
    robot.rarm_gripper_joint.joint_angle(angle)
    robot.rarm_mimic_gripper_joint.joint_angle(-angle)


real = copy.deepcopy(robot)  # 本物の姿勢を入れて手先を測るための模型


def hand_error():
    """robot と ri の右手先の距離 [m]。"""
    real.newcoords(ri.odom)
    real.angle_vector(ri.angle_vector())
    return np.linalg.norm(real.rarm_end_coords.worldpos()
                          - robot.rarm_end_coords.worldpos())


side = Coordinates()  # 手先は前向き

# A. 計画 (robot) が先、実行 (ri) が後
print('A. robot で IK を解く (青だけが動く)')
set_gripper(-1.0)
plan_arm(side.copy_worldcoords().translate((0.12, -0.08, 0.10), 'world'))
print('   手先のずれ {:.3f} m'.format(hand_error()))
print('   ri に送る')
send()
print('   手先のずれ {:.3f} m'.format(hand_error()))

# B. ベースを動かすのは ri だけ
print('B. ri.go_pos(0.30, 0, 0)')
ri.go_pos(0.30, 0.0, 0.0)
print('   robot の位置 x = {:.3f} m / ri.odom x = {:.3f} m'.format(
    robot.worldpos()[0], ri.odom.worldpos()[0]))
print('   このまま IK を解くと、0.30 m 後ろから手を伸ばす計画になる')
robot.newcoords(ri.odom)
print('   robot.newcoords(ri.odom) で青を本物の場所へ')

# 箱の後ろから手を差し込む。箱の位置は ri (センサ) から読む
p = ri.object_coords('cube').worldpos()
grasp = p + [0.0125, 0.0, 0.008]
plan_arm(side.copy_worldcoords().translate(grasp + [-0.05, 0, 0], 'world'))
send()
plan_arm(side.copy_worldcoords().translate(grasp, 'world'))
send()

# C. 指を閉じる: robot は 0.3 rad まで閉じたつもり、本物は箱で止まる
print('C. グリッパーを閉じる')
set_gripper(0.3)
send(1.0)
g = robot.joint_list.index(robot.rarm_gripper_joint)
print('   robot の指 {:.3f} rad / ri の指 {:.3f} rad (箱で止まった)'.format(
    robot.angle_vector()[g], ri.angle_vector()[g]))
print('   箱に触れているもの', ri.contacts('cube'))

plan_arm(side.copy_worldcoords().translate(grasp + [0, 0, 0.04], 'world'))
send()

# D. 箱の位置: シーンに置いた Box は置いた場所のまま、ri は今の場所
print('D. 箱の高さ: cube.worldpos() {:.3f} m / ri.object_coords {:.3f} m'
      .format(cube.worldpos()[2], ri.object_coords('cube').worldpos()[2]))
rise = ri.object_coords('cube').worldpos()[2] - p[2]
print('成功!' if rise > 0.02 else '失敗...')
