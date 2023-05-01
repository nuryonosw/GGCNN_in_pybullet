"""
Description:
    This is a test demo
    Press "p" to take a snapshot, close external window so that the robot will perform grasp action
    Press "q" to quit when robot is not performing grasp action

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
import cv2
import numpy as np

# ------------------------------------------    请输入pred_info进行测试    ------------------------------------------ #
pred_info = [0, 0, 90 * np.pi / 180, 80]  # 请输入pred_info


# pred_info = [pred_row, pred_col, pred_angle, pred_width_pix]
# img_h = 480, img_w = 640
# pred_row和世界坐标y轴平行， pred_col和世界坐标x轴平行
# 如果输入pred_row和pred_col都为0的话，机械臂末端是reach不过去的，相机画面足够大的

def creat_test_obj(bullet_client):
    bullet_client.loadURDF("lego/lego.urdf", basePosition=[-0.3849, 0.288675, 0.005],
                           globalScaling=1.3)


# ------------------------------------------    请输入pred_info进行测试    ------------------------------------------ #


p.connect(p.GUI)  # 连接服务器
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加pybullet_data的路径，以便load panda的urdf
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 不显示gui组件
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=160, cameraPitch=-30, cameraTargetPosition=[0.3, 0.05, 0.3])
p.setRealTimeSimulation(1)

timestep = 1. / (10 * 240.)
legos = m.creat_lego_env(p, 0)  # 创建环境
panda = pu.Panda(p, [0, -0.5, 0])  # 初始化Panda
cam = m.Camera(bullet_client=p)

creat_test_obj(bullet_client=p)


def show_snapshot():
    """
    展示rgb&dep 图像
    """
    cv2.destroyAllWindows()

    width, height, rgb_img, dep_img, seg_img = cam.get_img()  # snapshot
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)  # 将rgb排列变成bgr，因为cv2的convention是bgr
    dep_img_ = cv2.cvtColor(dep_img, cv2.COLOR_GRAY2BGR)  # # 将gray变成bgr，因为cv2的convention是bgr
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
    return dep_img  # 返回dep_img，以供do_grasp调用


def do_grasp(pred, dep_img):
    """
    输入预测得到的信息，完成单个物体抓取的流程
    :param pred: pred = [pred_row, pred_col, pred_angle, pred_width_pix], 预测得到的信息
                pred_row:像素的行数
                pred_col: 像素的列数
                pred_angle: 在图片中预测的抓取角度
                pred_width_pix: 在图片中预测的抓取宽度，单位是像素
    :param dep_img: 还原后的深度图
    """

    pred_row, pred_col, pred_angle, pred_width_pix = pred
    while True:
        grasp_pos = cam.get_grasp_pos(pix_pt=(pred_col, pred_row), depth=dep_img[pred_row, pred_col])
        grasp_angle = cam.get_grasp_angle(pix_grasp_angle=pred_angle)
        grasp_width = cam.get_grasp_width(pix_grasp_width=pred_width_pix, depth=dep_img[pred_row, pred_col])
        grasp_x, grasp_y, grasp_z = grasp_pos
        # 看起来额可以省略上面这一步，可以将pos直接传给60行的step函数，但这样会报错；由于pos是个tuple，不能指定元素，而在step函数中正好有这一个操作。
        p.stepSimulation()

        print("action status: " + str(panda.state) + "   \"-1 means a single grasp action is done.\" ")

        time.sleep(timestep)

        if panda.step([grasp_x, grasp_y, grasp_z], grasp_angle, grasp_width / 2):
            time.sleep(2)
            break
        panda.update_state()

        print('grasp_pos: ' + str([grasp_x, grasp_y, grasp_z]) +
              ' grasp_angle: ' + str(grasp_angle * np.pi / 180) + ' grasp_width: ' + str(grasp_width))


# ----------------------------------------    Begin Tesing    ---------------------------------------- #

snapshot = False

while 1:
    # if "q" is pressed, exit pybullet
    # if "p" is pressed, a snapshot img window will open, and close it to perform grasp action
    keys = p.getKeyboardEvents()
    key_p = ord('p')
    key_q = ord('q')
    if key_q in keys and keys[key_q] & p.KEY_WAS_TRIGGERED:
        break
    if key_p in keys and keys[key_p] & p.KEY_WAS_TRIGGERED:
        cv2.destroyAllWindows()
        snapshot = True

    if snapshot:
        # snspshot
        depimg = show_snapshot()
        # 进行抓取
        do_grasp(pred_info, depimg)

        snapshot = False

p.disconnect()
