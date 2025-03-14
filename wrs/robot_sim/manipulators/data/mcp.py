import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from ur3e.ur3e import UR3e
from manipulator_interface import ManipulatorInterface

def compute_tcp_positions(pkl_folder, max_points=2000):
    arm = UR3e(enable_cc=True)  # 根据需要传入参数
    tcp_positions = []  # 用于存储所有 TCP 位置
    manipulabilities = []  # 用于存储所有点的可操作性值

    total_tcp_count = 0  # 计数器，记录总的 TCP 数量

    for filename in os.listdir(pkl_folder):
        if filename.endswith('.pkl'):
            file_path = os.path.join(pkl_folder, filename)
            with open(file_path, 'rb') as f:
                joint_angles_list = pickle.load(f)

                for joint_angles in joint_angles_list:
                    if len(tcp_positions) >= max_points:  # 达到最大点数限制
                        break
                    if joint_angles.shape[0] == 5:
                        joint_angles = np.append(joint_angles, 0)
                    tcp_pos, rotation_matrix = arm.fk(joint_angles, update=True)  # 获取 TCP 和旋转矩阵
                    manipulability = arm.manipulability_val()  # 获取可操作性

                    # 只记录有效的可操作性值
                    if not np.isnan(manipulability):
                        tcp_positions.append(tcp_pos)
                        manipulabilities.append(manipulability)
                        total_tcp_count += 1  # 增加总 TCP 计数

    # 将 tcp_positions 和 manipulabilities 转换为 NumPy 数组
    tcp_positions = np.array(tcp_positions)
    manipulabilities = np.array(manipulabilities)

    # 打印整体可操作性范围
    if manipulabilities.size > 0:
        print(f"Overall Manipulability range: {np.min(manipulabilities)}, {np.max(manipulabilities)}")
    else:
        print("No valid manipulability values found.")

    print(f"Total TCP Positions Count: {total_tcp_count}")

    # 生成随机点
    random_points = np.random.rand(1000, 3)  # 在单位立方体内生成1000个随机点

    # 计算每个TCP位置对随机点的覆盖
    coverage = np.zeros(len(tcp_positions))
    for i, tcp_pos in enumerate(tcp_positions):
        for point in random_points:
            if np.linalg.norm(tcp_pos[:2] - point[:2]) <= 0.1:  # 假设覆盖半径为0.1
                if manipulabilities[i] > 0.15:  # 有效可操作性大于0.15
                    coverage[i] += 1

    # 选择覆盖最多的TCP位置
    max_coverage_index = np.argmax(coverage)
    print(f"TCP position with maximum coverage: {tcp_positions[max_coverage_index]}")
    print(f"Maximum coverage: {coverage[max_coverage_index]}")

    # 可视化所有 TCP 位置，并根据可操作性为所有点着色
    cmap = plt.cm.plasma
    max_manipulability = np.max(manipulabilities)
    num_bins = int(np.ceil(max_manipulability / 0.005)) + 1
    bounds = np.linspace(0, max_manipulability, num_bins)
    norm = BoundaryNorm(boundaries=bounds, ncolors=cmap.N, clip=True)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(tcp_positions[:, 0], tcp_positions[:, 1], tcp_positions[:, 2],
                         c=manipulabilities, cmap=cmap, norm=norm, alpha=0.7)
    cbar = fig.colorbar(scatter, ax=ax, boundaries=bounds)
    cbar.set_label('Manipulability')
    cbar.set_ticks(bounds)
    cbar.set_ticklabels([f"{b:.3f}" for b in bounds])
    ax.set_title('All TCP Positions Colored by Manipulability')
    ax.set_xlabel('X (dm)')
    ax.set_ylabel('Y (dm)')
    ax.set_zlabel('Z (dm)')
    plt.tight_layout()
    plt.show()

compute_tcp_positions('data/fk11')