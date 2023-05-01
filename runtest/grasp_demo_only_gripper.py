"""
Description:
    This is a test demo
    Panda gripper grasps objects in a constant motion routine
    Press "p" to start
    现在没办法抓起来，没找到原因

Auther:
    SeaHI-Robot:  https://github.com/SeaHI-Robot
    This code is for 2023Spring ME336 course project.
    For learning purposes only, do not spread without authorization.
    Contact_Me: seahirobot@gmail.com
"""

import pybullet as p
import pybullet_data
import utils.gripper_util as gu
import utils.misc as m
import time

timestep = 1. / 240.

p.connect(p.GUI)  # 连接服务器

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加pybullet_data的路径，以便load panda的urdf
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 不显示gui组件
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=160, cameraPitch=-30, cameraTargetPosition=[0.3, 0.05, 0.3])
p.setRealTimeSimulation(1)

legos = m.creat_lego_env(p, lego_num=3)  # 创建抓lego的环境
gripper = gu.Gripper(p, [0, 0, 0.5])  # 初始化gripper

# ----------------------------------------    Grasp Demo Runtime    ---------------------------------------- #

# 指定抓取角和抓取宽度
grasp_angle, grasp_width = (0, 0.08)

# 抓取
t = 0
which_lego = 0

while 1:
    keys = p.getKeyboardEvents()
    key_p = ord('p')
    if key_p in keys and keys[key_p] & p.KEY_WAS_TRIGGERED:
        break

# 仿真主循环
while t < 10000:

    pos, _ = p.getBasePositionAndOrientation(legos[which_lego])
    grasp_x, grasp_y, grasp_z = pos
    p.stepSimulation()
    t += 1
    print(gripper.state)

    time.sleep(timestep)

    # 设置base的位置
    gripper.reset_gripper([grasp_x, grasp_y, grasp_z + 0.205], grasp_angle, grasp_width / 2)

    if gripper.step(dist=0.1):
        p.removeBody(legos[which_lego])
        which_lego += 1
    gripper.update_state()

p.disconnect()
