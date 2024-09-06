import open3d as o3d
import numpy as np
import os
import time  

# 读取点云数据
denoised_cloud = o3d.io.read_point_cloud("cube_point_cloud.pcd")
o3d.visualization.draw_geometries([denoised_cloud])

# 打印原始数据点数
original_point_count = len(denoised_cloud.points)
print(f"原始数据点数: {original_point_count}")

# 初始化存储平面点云的列表
denoised_plane_clouds = []
plane_models = []

# 记录算法开始时间
start_time = time.time()

# 迭代进行平面分割
while True:
    # 进行平面分割
    current_plane_model, inliers = denoised_cloud.segment_plane(distance_threshold=0.05, ransac_n=3,
                                                                num_iterations=1000)

    # 如果找不到更多的平面，退出循环
    if len(inliers) < 1500:  # 根据实际情况调整阈值
        break

    # 提取平面上的点云
    denoised_plane_cloud = denoised_cloud.select_by_index(inliers)

    # 为平面点云赋予随机颜色
    random_color = np.random.rand(3)  # 生成随机颜色
    denoised_plane_cloud.paint_uniform_color(random_color)

    # 存储平面点云
    denoised_plane_clouds.append(denoised_plane_cloud)
    plane_models.append(current_plane_model)

    # 从点云中去除已经分割出的平面
    denoised_cloud = denoised_cloud.select_by_index(inliers, invert=True)

# 记录算法结束时间
end_time = time.time()

# 打印每个平面的颜色
for i, denoised_plane_cloud in enumerate(denoised_plane_clouds):
    color_array = np.asarray(denoised_plane_cloud.colors)[0]
    print(f"平面{i}的随机颜色: {color_array}")

# 可视化结果
o3d.visualization.draw_geometries(denoised_plane_clouds)

# 打印最终数据点数
final_point_count = sum(len(plane_cloud.points) for plane_cloud in denoised_plane_clouds)
print(f"最终数据点数: {final_point_count}")

# 打印算法耗时
elapsed_time = end_time - start_time
print(f"算法耗时: {elapsed_time:.2f} 秒")

# 保存每个平面点云到桌面
desktop_path = os.path.join(os.path.join(os.environ['USERPROFILE']), 'Desktop')
for i, plane_cloud in enumerate(denoised_plane_clouds):
    file_path = os.path.join(desktop_path, f"plane_{i}.pcd")
    o3d.io.write_point_cloud(file_path, plane_cloud)
    print(f"平面 {i} 已保存到 {file_path}")

# 合并所有平面点云为一个整体点云
combined_cloud = denoised_plane_clouds[0]
for plane_cloud in denoised_plane_clouds[1:]:
    combined_cloud += plane_cloud

# 保存整体点云到桌面
combined_file_path = os.path.join(desktop_path, "combined_planes.pcd")
o3d.io.write_point_cloud(combined_file_path, combined_cloud)
print(f"整体点云已保存到 {combined_file_path}")
