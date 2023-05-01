import numpy as np


# ------ start ------  creat_lego_env() | get_rand_pos() | get_trans_matrix()  ------ start ------ #


def creat_lego_env(bullet_client, lego_num):
    """
    :param bullet_client: pybullet client id
    :param lego_num: numer of lego in the environment
    :return:  a list that contains all the lego objects
    """

    bullet_client.loadURDF("plane.urdf")
    bullet_client.loadURDF("tray/traybox.urdf", basePosition=[0.5, 0, 0], globalScaling=0.5)
    bullet_client.setGravity(0, 0, -9.81)  # 设置重力
    legos = []
    x_range = [-0.18, 0.18]
    y_range = [-0.18, 0.18]
    z_range = [0.1, 0.2]
    base_points = get_rand_pos(x_range, y_range, z_range, n=lego_num)

    for i in range(lego_num):
        import random
        legos.append(bullet_client.loadURDF("lego/lego.urdf", basePosition=base_points[i],
                                            globalScaling=random.uniform(1, 1.5)))
        rand_rgba = [random.uniform(0.3, 1), random.uniform(0.3, 1), random.uniform(0.3, 1), 1]
        bullet_client.changeVisualShape(objectUniqueId=legos[i], linkIndex=-1,
                                        rgbaColor=rand_rgba)

    return legos


def get_rand_pos(x_range, y_range, z_range, n=1):
    """
    生成n组随机三维坐标,x坐标在x_range范围内,y坐标在y_range范围内,
    """
    import random
    points = []
    for i in range(n):
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        z = random.uniform(z_range[0], z_range[1])
        points.append((x, y, z))
    return points


def get_trans_matrix(euler_angle, pos_offset):
    """
    输入欧拉角和位移的偏置，输出4x4的变换矩阵
    get_trans_matrix函数仅在Camera类中被使用
    :param euler_angle:欧拉角[r,p,y], 绕世界坐标系xyz旋转，先x再y后z，左乘
    :param pos_offset: 位置offset
    :return: 4x4 Trasformation Matrix
    """
    # 从欧拉角获得旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(euler_angle[0]), -np.sin(euler_angle[0])],
                    [0, np.sin(euler_angle[0]), np.cos(euler_angle[0])]
                    ])

    R_y = np.array([[np.cos(euler_angle[1]), 0, np.sin(euler_angle[1])],
                    [0, 1, 0],
                    [-np.sin(euler_angle[1]), 0, np.cos(euler_angle[1])]
                    ])

    R_z = np.array([[np.cos(euler_angle[2]), -np.sin(euler_angle[2]), 0],
                    [np.sin(euler_angle[2]), np.cos(euler_angle[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))

    trans_matrix = np.array([
        [R[0, 0], R[0, 1], R[0, 2], pos_offset[0]],
        [R[1, 0], R[1, 1], R[1, 2], pos_offset[1]],
        [R[2, 0], R[2, 1], R[2, 2], pos_offset[2]],
        [0, 0, 0, 1.]
    ])

    return trans_matrix


# ------  end  ------  creat_lego_env() | get_rand_pos() | get_trans_matrix()  ------  end  ------ #


# ------ start -------------------------   Class Camera   ------------------------  start  ------ #
class Camera:
    def __init__(self, bullet_client, eye_pos=None, targ_pos=None, up_vec=None, img_w=640, img_h=480):
        """
        初始化pybullet仿真相机的参数，并计算相机内参
        :param eye_pos 默认 [0, 0, 0.5]
        :param targ_pos 默认 [0, 0, 0]
        :param up_vec 默认 [0, 1, 0]
        """

        if eye_pos is None:  # 相机位置
            eye_pos = [0, 0, 0.5]  # 默认值
        if targ_pos is None:  # 目标位置
            targ_pos = [0, 0, 0]  # 默认值
        if up_vec is None:  # 相机顶部朝向
            up_vec = [0, 1, 0]  # 默认值

        self.eye_pos = eye_pos

        fov = 60  # fov为视场角，此时fov60的垂直视场： 图像高tan(30) * 0.7 *2 = 0.8082903m
        aspect = img_w / img_h  # aspect ratio,纵横比,宽高比
        self.near_val = 0.1  # 最近有效距离, 在get_image中需要被调用
        self.far_val = 1  # 最远有效距离, 在get_image中需要被调用

        self.img_w = img_w
        self.img_h = img_h
        self.p = bullet_client
        self.view_matrix = self.p.computeViewMatrix(eye_pos, targ_pos, up_vec)
        self.projection_matrix = self.p.computeProjectionMatrixFOV(fov, aspect, self.near_val, self.far_val)

        # 计算内参矩阵 http://ksimek.github.io/2013/08/13/intrinsic/
        targ_dep = eye_pos[2] - targ_pos[2]  # targ平面到相机的深度
        real_world_half_height = targ_dep * np.tan((fov / 2) * np.pi / 180)  # 目标平面的1/2真实高度
        self.focal_length = (img_h / 2) * targ_dep / real_world_half_height  # 这里认为fx和fy是相等的，是理想的针孔相机
        f = self.focal_length
        self.intrinsic_matrix = np.array([[f, 0, self.img_w / 2 - 0.],
                                          [0, f, self.img_h / 2 - 0.],
                                          [0, 0, 1]],
                                         dtype=np.float)
        # 上面内参矩阵中主轴偏置x0和y0在参考代码里减去了0.5，个人理解是考虑到像素的平面尺寸的影响
        # 但实际测试减去0.5会有一定偏差，不减去的话实测反而表现很准：如输入像素坐标为 1/2的img_w和1/2的img_h时，得到的grasp_pos的xy坐标基本就是0
        self.trans_matrix = get_trans_matrix(euler_angle=[np.pi, 0, 0], pos_offset=self.eye_pos)

    def get_img(self):
        """
        调用get_img()会进行一次拍摄并返回当前图像信息的tuple:
        :return  width, height, rgbImg, depthImg, segImg
        """

        """""
        关于p.getCameraImage:
        "(width, height, rgbImg, depthImg, segImg) = p.getCameraImage()"
        此处返回的rgbImage是四维的rgba格式
        depthImg不是真正的深度图，是一个depth，"buffer"，参照官方文档经过操作可以得到真实深度图
        segImg是一个segment mask buffer，也需要一定操作还原
        以下是参照官方文档，从depth buffer获取真实深度图像的操作
        far=1000.//depends on projection matrix, this is default
        near=0.01//depends on projection matrix
        depth = far * near / (far - (far - near) * depthImg) //depthImg is the depth from Bullet 'getCameraImage'
        """""

        width, height, rgbImg, depthImg, segImg = self.p.getCameraImage(
            width=self.img_w, height=self.img_h,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix)
        rgbImg = rgbImg[:, :, :3]  # 取rgb图像
        depthImg = self.far_val * self.near_val / (self.far_val - (self.far_val - self.near_val) * depthImg)

        return width, height, rgbImg, depthImg, segImg

    def get_grasp_pos(self, pix_pt, depth):
        """
        把图像中的二维像素坐标点，转化成世界坐标中的三维点
        :param pix_pt: [x, y] 图片中二维像素点坐标
        :param depth: 深度值
        :return 世界坐标系中的三维坐标
        """
        pt_in_img = np.array([[pix_pt[0]], [pix_pt[1]], [1]], dtype=np.float)  # 把[x,y]变成[x,y,1]
        pt_in_cam_coor = np.matmul(np.linalg.inv(self.intrinsic_matrix), pt_in_img) * depth  # 像素点对应的在相机系中的坐标
        pt_in_cam_coor = list(pt_in_cam_coor.reshape((3,)))  # 转换成list
        pt_in_cam_coor.append(1.)  # 三维升一维变四维
        pt_in_cam_coor = np.array(pt_in_cam_coor).reshape((4, 1))  # reshape一下
        pt_in_world_coor = np.matmul(self.trans_matrix, pt_in_cam_coor).reshape((4,))  # 变换矩阵处理一下
        pt_in_world_coor = list(pt_in_world_coor)[:-1]  # 转换成list并取x,y,z

        return pt_in_world_coor

    def get_grasp_angle(self, pix_grasp_angle):
        """
        在摄像头冲着y轴正方向且垂直地面向下拍摄时，认为像素中的grasp_angle和实际的grasp_angle是一样的
        若摄像头并不是保持这样的理想位姿，则需要对这个函数做修改，有待解决。
        :return real_world_grasp
        """
        real_world_grasp_angle = pix_grasp_angle
        return real_world_grasp_angle

    def get_grasp_width(self, pix_grasp_width, depth):
        """
        输入像素长度，输出真实世界的长度
        :param pix_grasp_width: 像素长度
        :param depth: 真实世界的深度
        :return 像素尺寸对应的真实世界的尺寸
        """
        grasp_width = pix_grasp_width * depth / self.focal_length
        if grasp_width > 0.08:
            return 0.08
        else:
            return grasp_width
# ------  end  -------------------------   Class Camera   ------------------------   end   ------ #
