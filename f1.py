import pandas as pd
import numpy as np
from scipy.spatial import KDTree
from sklearn.metrics import precision_score, recall_score, f1_score

# 文件名
predicted_file = "13.csv"
ground_truth_file = "plane_2.csv"

# 打印文件名
print(f"Predicted file: {predicted_file}")
print(f"Ground truth file: {ground_truth_file}")

# 读取预测数据CSV文件和真实数据CSV文件
predicted_labels = pd.read_csv(f"./data/{predicted_file}")
ground_truth_labels = pd.read_csv(f"./data/{ground_truth_file}")

# 提取点云数据
predicted_points = predicted_labels[['x', 'y', 'z']].values
ground_truth_points = ground_truth_labels[['x', 'y', 'z']].values

# 使用KDTree找到每个预测点的最近邻真实点
tree = KDTree(ground_truth_points)
distances, indices = tree.query(predicted_points)

# 设置极小的距离阈值
distance_threshold = 1e-5

# 找到有效匹配的点对，使用极小的阈值
valid_matches = distances < distance_threshold

# 计算有效匹配的点数
num_valid_matches = np.sum(valid_matches)

# 计算匹配的比例
precision = num_valid_matches / len(predicted_points)
recall = num_valid_matches / len(ground_truth_points)
f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

# 输出结果
print(f"Precision: {precision}")
print(f"Recall: {recall}")
print(f"F1 Score: {f1}")
print(f"Valid matches: {num_valid_matches}")
print(f"Predicted points: {len(predicted_points)}")
print(f"Ground truth points: {len(ground_truth_points)}")
