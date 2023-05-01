"""
Description:
    This is a test demo
    Press "p" to take a snapshot, then you will see a 3d depth image as well as a composition of rgb&depth img
    Press "q" to terminate

Auther:
    SeaHI-Robot:  https://github.com/SeaHI-Robot
    This code is for 2023Spring ME336 course project.
    For learning purposes only, do not spread without authorization.
    Contact_Me: seahirobot@gmail.com
"""

import time
import pybullet as p
import pybullet_data
import utils.misc as m
import cv2
import matplotlib.pyplot as plt
import numpy as np

p.connect(p.GUI)  # 连接服务器
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加pybullet_data的路径，以便load panda的urdf
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # 显不显示示gui组件
p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=160, cameraPitch=-30, cameraTargetPosition=[0., 0.05, 0.3])
p.setRealTimeSimulation(1)
timestep = 1. / 240.

legos = m.creat_lego_env(p, 5)  # 创建抓lego的环境
cam = m.Camera(bullet_client=p)
snapshot = False

while 1:
    p.stepSimulation()
    time.sleep(timestep)

    keys = p.getKeyboardEvents()
    key_p = ord('p')
    key_q = ord('q')
    # press "q" to take a snapshot, the visualization will be presented,
    # remember to close the external window before taking next snapshot!
    if key_p in keys and keys[key_p] & p.KEY_WAS_TRIGGERED:
        cv2.destroyAllWindows()
        plt.close()
        snapshot = True
    # press "q" to terminate
    if key_q in keys and keys[key_q] & p.KEY_WAS_TRIGGERED:
        break

    if snapshot:
        width, height, rgb_img, dep_img, seg_img = cam.get_img()
        # 展示3d深度图，z=0平面与地面平齐
        x = np.linspace(1, height, height)
        y = np.linspace(1, width, width)
        x_, y_ = np.meshgrid(x, y, indexing='ij')
        fig = plt.figure(facecolor='white')  # 创建图片
        sub = fig.add_subplot(111, projection='3d')  # 添加子图，
        surf = sub.plot_surface(x_, y_, 0.5 - dep_img, cmap=plt.cm.brg)  # 绘制曲面,cmap=plt.cm.brg并设置颜色cmap
        cb = fig.colorbar(surf, shrink=0.8, aspect=15)  # 设置颜色棒
        sub.set_xlabel(r"img_height")
        sub.set_ylabel(r"img_width")
        sub.set_zlabel(r"pixel depth")
        plt.title("Depth data on z axis, where z=0 is gound plane")
        plt.show()

        #  展示rgb&dep 图像
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)  # 将rgb排列变成bgr，因为cv2的convention是bgr
        dep_img_ = cv2.cvtColor(dep_img, cv2.COLOR_GRAY2BGR)
        disp_img = np.zeros((2 * height, width, 3), np.uint8)
        disp_img[:height, :width] = rgb_img
        disp_img[height:, :width] = (dep_img_ * 255).astype(np.uint8)
        # 原图片直接imshow出来有点大，把图像缩小一杯再imshow一下
        h, w = disp_img.shape[:2]
        w = int(0.5 * w)
        h = int(0.5 * h)
        disp_img = cv2.resize(disp_img, dsize=(w, h), fx=0.5, fy=0.5)
        cv2.imshow('rgb&dep img', disp_img)
        cv2.waitKey(0)

        snapshot = False

p.disconnect()
