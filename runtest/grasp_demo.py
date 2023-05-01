"""
Description:
    This is a test demo
    Panda robot grasps objects in a constant motion routine
    Press "p" to start

Auther:
    SeaHI-Robot:  https://github.com/SeaHI-Robot
    This code is for 2023Spring ME336 course project.
    For learning purposes only, do not spread without authorization.
    Contact_Me: seahirobot@gmail.com
"""

import pybullet as p
import pybullet_data
import utils.panda_util as pu
import utils.misc as m
import time

timestep = 1. / (10 * 240.)

p.connect(p.GUI)  # 连接服务器
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加pybullet_data的路径，以便load panda的urdf
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 不显示gui组件
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=160, cameraPitch=-30, cameraTargetPosition=[0.3, 0.05, 0.3])
p.setRealTimeSimulation(1)

legos = m.creat_lego_env(p, 5)  # 创建抓lego的环境
panda = pu.Panda(p, [0, -0.5, 0])  # 初始化Panda

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
    grasp_x, grasp_y, grasp_z = pos  # 看起来额可以省略，可以将pos直接传给60行的step函数，但这样会报错；由于pos是个tuple，不能指定元素，而在step函数中正好有这一个操作。
    p.stepSimulation()
    t += 1
    print("action status: "+str(panda.state)+"      \"-1 means a single grasp action is done.\" ")

    time.sleep(timestep)

    if panda.step([grasp_x, grasp_y, grasp_z], grasp_angle, grasp_width / 2):
        which_lego += 1
    panda.update_state()

p.disconnect()
