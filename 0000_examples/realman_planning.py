from wrs import wd, rm, mgm  # 根据需要替换 wrs 的具体模块
from wrs.robot_sim.manipulators.realman_arm.realmanArm import Realman  # 替换为实际的 realman_arm 模型类
from wrs.robot_con.realman.realman import RealmanArmController
import numpy as np

# 初始化仿真环境
base = wd.World(cam_pos=rm.vec(2, 0, 1), lookat_pos=rm.vec(0, 0, 0.5))  # 摄像机位置可调整
mgm.gen_frame().attach_to(base)

# 初始化 RealMan 机械手
robot = Realman(enable_cc=True)  # 替换为 realman_arm 机械手的初始化方法

# 初始化 RealManArmController 控制器实例
realman_controller = RealmanArmController(ip="192.168.1.18", port=8080, has_gripper=False)

# 存储机械臂模型节点
plot_node = [None]

def plot_rbt_realtime(task):
    """
    更新仿真环境中的机械臂状态，确保仿真中的机械臂与现实中的机械臂同步。
    """
    # 获取现实机械臂的关节角度（度数）
    _, joint_angles_deg = realman_controller.get_joint_values()  # 获取当前机械臂的关节角度，丢弃第一个返回值（状态码）

    # 确保 joint_angles_deg 是 numpy 数组类型
    joint_angles_deg = np.array(joint_angles_deg)

    # 检查关节角度是否是正确的一维数组
    if joint_angles_deg.ndim != 1:
        print(f"错误：关节角度数据格式不正确，获得的数据形状是 {joint_angles_deg.shape}")
        return task.again

    # 将关节角度从度数转换为弧度
    joint_angles_rad = np.radians(joint_angles_deg)

    # 更新仿真机械臂，使用弧度值
    robot.goto_given_conf(joint_angles_rad)  # 使用弧度值更新仿真机械臂

    # 重新生成机械臂模型并附加到仿真环境
    if plot_node[0] is not None:
        plot_node[0].detach()  # 移除之前的机械臂模型
    plot_node[0] = robot.gen_meshmodel()  # 生成新的机械臂模型
    plot_node[0].attach_to(base)  # 将机械臂模型附加到仿真环境中

    return task.again  # 保持实时更新


# 启动任务管理器，0.1秒更新一次机械臂模型
base.taskMgr.doMethodLater(0.1, plot_rbt_realtime, "update robot state")

# 启动仿真环境
base.run()

# 最后在程序结束时关闭连接
realman_controller.close_connection()
