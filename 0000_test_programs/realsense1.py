# import numpy as np
# import wrs.drivers.devices.realsense.realsense_d400s as rs
# from wrs import wd, rm, mgm
# from wrs.robot_con.realman_arm.realman_arm import RealmanArmController  # 导入 RealmanArmController 类
#
# # 假设你已经得到了旋转矩阵、平移向量和四元数
# rotation_matrix = np.array([
#     [-0.02521705, -0.9995292, 0.01747804],
#     [0.9994405, -0.02559138, -0.02153546],
#     [0.02197261, 0.01692521, 0.9996153]
# ])
#
# translation_vector = np.array([[0.09378063], [-0.02335285], [0.02118068]])
#
# quaternion = np.array([0.01377534, -0.00160981, 0.71596504, 0.69799836])
#
#
# # 假设您有一个RealSense相机对象
# class RealSenseD405WithCalibration(rs.RealSenseD405):
#     def __init__(self):
#         super().__init__()
#         # 将旋转矩阵和平移向量作为相机的内外参
#         self._init_calib_mat = np.eye(4)
#         self._init_calib_mat[:3, :3] = rotation_matrix  # 将旋转矩阵赋值给变换矩阵
#         self._init_calib_mat[:3, 3] = translation_vector.ravel()  # 将平移向量赋值给变换矩阵的平移部分
#
#         # 初始化机器人控制器
#         self.realman_controller = RealmanArmController(ip="192.168.1.18", port=8080, has_gripper=False)
#
#     def align_pcd(self, pcd):
#         # 使用 RealmanArmController 实例获取机器人的位姿
#         rbt_pose = self.realman_controller.get_pose()  # 获取位姿
#         w2r_mat = rm.homomat_from_posrot(*rbt_pose)  # 通过位姿生成旋转矩阵和平移向量
#         w2c_mat = w2r_mat.dot(self._init_calib_mat)  # 将旋转和平移矩阵进行合并
#         return rm.transform_points_by_homomat(w2c_mat, points=pcd)
#
#     def close_connection(self):
#         self.realman_controller.close_connection()  # 关闭连接
#
#
# # 创建具有校准矩阵的相机对象
# d405 = RealSenseD405WithCalibration()
#
# # 在仿真环境中运行
# base = wd.World(cam_pos=rm.vec(2, 1, 1), lookat_pos=rm.vec(0, 0, 0))
# mgm.gen_frame().attach_to(base)
#
# onscreen = []
#
#
# def update(d405, onscreen, task):
#     if len(onscreen) > 0:
#         for ele in onscreen:
#             ele.detach()
#     pcd, pcd_color = d405.get_pcd(return_color=True)
#     onscreen.append(mgm.gen_pointcloud(pcd, pcd_color))
#     onscreen[-1].attach_to(base)
#     return task.cont
#
#
# base.taskMgr.add(update, "update", extraArgs=[d405, onscreen], appendTask=True)
# base.run()


import numpy as np
import wrs.drivers.devices.realsense.realsense_d400s as rs
from wrs import wd, rm, mgm
from wrs.robot_con.realman.realman import RealmanArmController  # 导入 RealmanArmController 类
from wrs.robot_sim.manipulators.realman_arm.realmanArm import Realman

# 假设你已经得到了旋转矩阵、平移向量和四元数
rotation_matrix = np.array([
    [-0.02521705, -0.9995292, 0.01747804],
    [0.9994405, -0.02559138, -0.02153546],
    [0.02197261, 0.01692521, 0.9996153]
])

translation_vector = np.array([[0.09378063], [-0.02335285], [0.02118068]])

quaternion = np.array([0.01377534, -0.00160981, 0.71596504, 0.69799836])


# 假设您有一个RealSense相机对象
class RealSenseD405WithCalibration(rs.RealSenseD405):
    def __init__(self):
        super().__init__()
        # 将旋转矩阵和平移向量作为相机的内外参
        self._init_calib_mat = np.eye(4)
        self._init_calib_mat[:3, :3] = rotation_matrix  # 将旋转矩阵赋值给变换矩阵
        self._init_calib_mat[:3, 3] = translation_vector.ravel()  # 将平移向量赋值给变换矩阵的平移部分

        # 初始化机器人控制器
        self.realman_controller = RealmanArmController(ip="192.168.1.18", port=8080, has_gripper=False)

    def align_pcd(self, pcd):
        # 使用 RealmanArmController 实例获取机器人的位姿
        rbt_pose = self.realman_controller.get_pose()  # 获取位姿
        w2r_mat = rm.homomat_from_posrot(*rbt_pose)  # 通过位姿生成旋转矩阵和平移向量
        w2c_mat = w2r_mat.dot(self._init_calib_mat)  # 将旋转和平移矩阵进行合并
        return rm.transform_points_by_homomat(w2c_mat, points=pcd)

    def close_connection(self):
        self.realman_controller.close_connection()  # 关闭连接


# 创建具有校准矩阵的相机对象
d405 = RealSenseD405WithCalibration()

# 在仿真环境中运行
base = wd.World(cam_pos=rm.vec(2, 1, 1), lookat_pos=rm.vec(0, 0, 0))
mgm.gen_frame().attach_to(base)  # 添加坐标框架（可选）

# 创建机械臂模型并附加到仿真环境
arm = RealmanArmController(ip="192.168.1.18", port=8080, has_gripper=False)
arm_mesh = arm.gen_meshmodel(alpha=.3)  # 生成机械臂的网格模型
arm_mesh.attach_to(base)  # 将机械臂网格附加到仿真世界中

onscreen = []  # 用于存储点云数据


# 更新函数：每帧更新相机点云并将其显示在仿真中
def update(d405, onscreen, task):
    if len(onscreen) > 0:
        for ele in onscreen:
            ele.detach()  # 清除先前的点云显示

    pcd, pcd_color = d405.get_pcd(return_color=True)  # 获取点云数据
    onscreen.append(mgm.gen_pointcloud(pcd, pcd_color))  # 生成新的点云对象
    onscreen[-1].attach_to(base)  # 将点云附加到仿真世界中
    return task.cont


base.taskMgr.add(update, "update", extraArgs=[d405, onscreen], appendTask=True)

# 启动仿真
base.run()

