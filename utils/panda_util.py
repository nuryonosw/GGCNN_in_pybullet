"""
Description:
    This script programs how panda robot moves when grasping

Auther:
    SeaHI-Robot:  https://github.com/SeaHI-Robot
    This code is for 2023Spring ME336 course project.
    For learning purposes only, do not spread without authorization.
    Contact_Me: seahirobot@gmail.com
"""

import numpy as np
import math

pandaEndEffectorIndex = 11
pandaNumDofs = 7

ll = [-7] * pandaNumDofs
# upper limits for null space (todo: set them to proper range)
ul = [7] * pandaNumDofs
# joint ranges for null space (todo: set them to proper range)
jr = [7] * pandaNumDofs
# restposes for null space

# 7个关节joint,两个夹爪joint
jointPositions = (
    0.8, 0.5, 0.0, -1.0, 0.0,
    1.5, 0.0, 0.0, 0.0)

# rest pos
rp = jointPositions


class PandaSim(object):
    def __init__(self, bullet_client, offset):
        self.cur_state = None
        self.state_t = None
        self.p = bullet_client
        self.p.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)

        flags = self.p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        orn = [0, 0, 0, 1]

        self.pandaId = self.p.loadURDF("franka_panda/panda.urdf", np.array([0, 0, 0]) + self.offset, orn,
                                       useFixedBase=True, flags=flags)
        index = 0
        self.state = 0
        self.control_dt = 1. / 240.
        self.finger_target = 0
        self.gripper_height = 0.2
        # create a constraint to keep the fingers centered
        c = self.p.createConstraint(self.pandaId,
                                    9,
                                    self.pandaId,
                                    10,
                                    jointType=self.p.JOINT_GEAR,
                                    jointAxis=[1, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
        self.p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

        for j in range(self.p.getNumJoints(self.pandaId)):
            self.p.changeDynamics(self.pandaId, j, linearDamping=0, angularDamping=0)
            info = self.p.getJointInfo(self.pandaId, j)

            jointType = info[2]
            if jointType == self.p.JOINT_PRISMATIC:
                self.p.resetJointState(self.pandaId, j, jointPositions[index])
                index = index + 1

            if jointType == self.p.JOINT_REVOLUTE:
                self.p.resetJointState(self.pandaId, j, jointPositions[index])
                index = index + 1
        self.t = 0.

    def compute_jointpos(self, pos, orn):
        """
        根据 pos 和 orn 计算机械臂的关节位置
        """
        jointPoses = self.p.calculateInverseKinematics(self.pandaId, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp,
                                                       maxNumIterations=20)
        return jointPoses

    def set_arm(self, pos):
        orn = self.p.getQuaternionFromEuler([math.pi, 0., math.pi / 2])  # 机械手方向
        jointPoses = self.compute_jointpos(pos, orn)
        self.set_jointconfig(jointPoses)

    def set_jointconfig(self, jointPoses, maxVelocity=10.0):
        """
        分别设置7个关节的关节角
        """
        for i in range(pandaNumDofs):  # 7
            self.p.setJointMotorControl2(self.pandaId, i, self.p.POSITION_CONTROL, jointPoses[i], force=5 * 240.,
                                         maxVelocity=maxVelocity)

    def set_gripperconfig(self, finger_target):
        """
        设置机械手位置
        """
        for i in [9, 10]:
            self.p.setJointMotorControl2(self.pandaId, i, self.p.POSITION_CONTROL, finger_target, force=20)

    def step(self, pos, angle, gripper_w):
        """
        执行单次抓取的任务流程
        :param pos: [x, y, z] 机械手末端的位置
                x: 世界坐标系x轴，沿机械臂右侧方向
                y: 世界坐标系y轴，沿机械臂前方方向
                y: 世界坐标系z轴，沿机械臂上方方向
                单位米，原点位于托盘中心
        :param angle: 弧度
        :param gripper_w: 抓取器张开宽度
        :return True or False, 代表单次抓取流程是否执行完成
        """

        if self.state == 0:
            # print('恢复初始状态')
            pos[2] = 0.2
            orn = self.p.getQuaternionFromEuler([math.pi, 0., angle + math.pi / 2])  # 机械手方向
            jointPoses = self.compute_jointpos(pos, orn)
            self.set_jointconfig(jointPoses)
            self.set_gripperconfig(gripper_w)
            return False

        elif self.state == 1:
            # print('物体上方')
            pos[2] += 0.08
            orn = self.p.getQuaternionFromEuler([math.pi, 0., angle + math.pi / 2])  # 机械手方向
            jointPoses = self.compute_jointpos(pos, orn)
            self.set_jointconfig(jointPoses)
            return False

        elif self.state == 2:
            # print('达到抓取位置')

            orn = self.p.getQuaternionFromEuler([math.pi, 0., angle + math.pi / 2])  # 机械手方向
            jointPoses = self.compute_jointpos(pos, orn)
            self.set_jointconfig(jointPoses, maxVelocity=8.)
            return False

        elif self.state == 3:
            # print('闭合抓取器')
            self.set_gripperconfig(0)
            return False

        elif self.state == 4:
            # print('物体上方(预抓取位置)')
            pos[2] = 0.2
            orn = self.p.getQuaternionFromEuler([math.pi, 0., angle + math.pi / 2])  # 机械手方向
            jointPoses = self.compute_jointpos(pos, orn)
            self.set_jointconfig(jointPoses, maxVelocity=2.)

            return False

        elif self.state == 5:
            # print('放置位置')
            pos = [0.5, 0., 0.2]

            orn = self.p.getQuaternionFromEuler([math.pi, 0., angle + math.pi / 2])  # 机械手方向
            jointPoses = self.compute_jointpos(pos, orn)
            self.set_jointconfig(jointPoses, maxVelocity=5.)

            return False

        elif self.state == 6:
            # print('张开抓取器')
            self.set_gripperconfig(0.08)
            return False

        elif self.state == -1:
            self.reset()  # 重置
            return True

    def reset(self):
        """
        重置状态
        """
        self.state = 0
        self.state_t = 0
        self.cur_state = 0


class Panda(PandaSim):
    def __init__(self, bullet_client, offset):
        PandaSim.__init__(self, bullet_client, offset)
        self.state_t = 0  # state_time,在一个state持续的时间
        self.cur_state = 0  # 当前state的序号
        self.states = [0, 1, 2, 3, 4, 5, 6, -1]  # 储存不同state的id
        self.state_durations = [0.3, 0.2, 0.2, 0.15, 0.3, 0.3, 0.3, 0.2]  # # 储存不同state下的持续时间

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]
