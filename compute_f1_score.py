import pandas as pd
import numpy as np
from scipy.spatial import KDTree
from sklearn.metrics import precision_score, recall_score, f1_score

# File names for predicted and ground truth data
predicted_file = "13.csv"
ground_truth_file = "plane_2.csv"

# Print the file names
print(f"Predicted file: {predicted_file}")
print(f"Ground truth file: {ground_truth_file}")

# Read the predicted data CSV file and the ground truth data CSV file
predicted_labels = pd.read_csv(f"./data/{predicted_file}")
ground_truth_labels = pd.read_csv(f"./data/{ground_truth_file}")

# Extract point cloud data (x, y, z coordinates)
predicted_points = predicted_labels[['x', 'y', 'z']].values
ground_truth_points = ground_truth_labels[['x', 'y', 'z']].values

# Build a KDTree using the ground truth points for nearest neighbor search
tree = KDTree(ground_truth_points)

# Query the KDTree to find the nearest neighbor in ground truth for each predicted point
distances, indices = tree.query(predicted_points)

# Set a very small distance threshold to determine valid matches
distance_threshold = 1e-5

# Find valid matches where the distance is smaller than the threshold
valid_matches = distances < distance_threshold

# Calculate the number of valid matches (predicted points that have a close ground truth match)
num_valid_matches = np.sum(valid_matches)

# Calculate precision as the ratio of valid matches to the total number of predicted points
precision = num_valid_matches / len(predicted_points)

# Calculate recall as the ratio of valid matches to the total number of ground truth points
recall = num_valid_matches / len(ground_truth_points)

# Calculate the F1 score, which is the harmonic mean of precision and recall
# F1 score is only calculated if precision + recall is greater than 0 to avoid division by zero
f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

# Output the results including precision, recall, F1 score, and other details
print(f"Precision: {precision}")
print(f"Recall: {recall}")
print(f"F1 Score: {f1}")
print(f"Valid matches: {num_valid_matches}")
print(f"Predicted points: {len(predicted_points)}")
print(f"Ground truth points: {len(ground_truth_points)}")
