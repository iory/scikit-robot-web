# Jedy で、低い台の上の箱を横からつかむ。
# Jedy は背が 30 cm ほどなので、台も箱も小さい。
import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.interfaces import MujocoRobotInterface
from skrobot.model import Box


TABLE_HEIGHT = 0.065
CUBE = 0.035  # 閉じた指の棒の間 (約 3.8 cm) に収まる大きさ

WHEELS = {  # 関節名 -> (半径 [m], ローラーの角度 [rad])。01_basics を参照
    'front_left_wheel_joint': (0.03, -np.pi / 4),
    'front_right_wheel_joint': (0.03, np.pi / 4),
    'rear_left_wheel_joint': (0.03, np.pi / 4),
    'rear_right_wheel_joint': (0.03, -np.pi / 4),
}
# 指は歯車と棒でできた凹んだ形。MuJoCo はメッシュの当たり判定を凸包で
# 行うので、そのままでは指の間が埋まってつかめない。指のリンクだけ凸な
# 部品に分割する (ブラウザ用には事前に計算してある)。
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
    """右手先を coords へ。IK が解けなければ止める。"""
    ok = robot.inverse_kinematics(
        coords,
        link_list=robot.rarm.link_list,
        move_target=robot.rarm_end_coords,
        stop=300)  # 反復回数の上限
    if ok is False:
        raise RuntimeError('IK が解けませんでした')
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()


def gripper(angle, time=1.0):
    """右グリッパー: 0 で閉じ、負にすると開く (左右の指は連動)。"""
    robot.rarm_gripper_joint.joint_angle(angle)
    robot.rarm_mimic_gripper_joint.joint_angle(-angle)
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()


# 手先は前向き (x 軸が前、指は左右から挟む)
side = Coordinates()

# 1. 手を開いて、台より少し高い所に構える
gripper(-1.0)
move_arm(side.copy_worldcoords().translate((0.12, -0.08, 0.10), 'world'))

# 2. 台の手前まで進む
ri.go_pos(0.30, 0.0, 0.0)
# ri が動かすのはシミュレーションの中のロボットだけ。robot (模型) も
# 今いる場所へ移しておかないと、IK は元の場所から手を伸ばそうとする。
robot.newcoords(ri.odom)

# 3. 箱の後ろから手を差し込み、挟んで持ち上げる
# 手先を箱の中心より少し前に出して、箱を指の棒の真ん中で挟む。
# 指は上下に 4 cm 厚みがあるので、中心より少し上をつかむ
# (中心の高さだと指の下端が台に当たる)。
p = ri.object_coords('cube').worldpos()  # 箱の今の位置を ri から読む
grasp = p + [0.0125, 0.0, 0.008]
move_arm(side.copy_worldcoords().translate(grasp + [-0.05, 0, 0], 'world'))
move_arm(side.copy_worldcoords().translate(grasp, 'world'))
gripper(0.3)
print('箱に触れているもの', ri.contacts('cube'))
move_arm(side.copy_worldcoords().translate(grasp + [0, 0, 0.04], 'world'))

rise = ri.object_coords('cube').worldpos()[2] - p[2]
print('持ち上げた高さ {:.3f} m'.format(rise))
print('成功!' if rise > 0.02 else '失敗...')
