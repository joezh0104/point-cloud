import matplotlib.pyplot as plt
import numpy as np

# Data for precision and recall across the planes
# Lists contain the precision and recall values for different algorithms and planes
planes = ['Plane1', 'Plane2', 'Plane3', 'Plane4']
precision_rg = [0.876, 0.847, 0.860, 0.742]
recall_rg = [0.882, 0.878, 0.861, 0.913]

precision_ransac_pcl = [0.889, 0.722, 0.782, 0.789]
recall_ransac_pcl = [0.856, 0.893, 0.355, 0.505]

precision_ransac_o3d = [0.888, 0.754, 0.842, 0.673]
recall_ransac_o3d = [0.854, 0.893, 0.406, 0.458]

# Create subplots
# Create a figure with 1 row and 2 columns for side-by-side precision and recall plots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Plot Precision
# Plot precision for RG_PCL, RANSAC_PCL, and RANSAC_O3D with markers for each plane
ax1.plot(planes, precision_rg, label='RG_PCL', marker='o', color='#4C72B0')
ax1.plot(planes, precision_ransac_pcl, label='RANSAC_PCL', marker='o', color='#55A868')
ax1.plot(planes, precision_ransac_o3d, label='RANSAC_O3D', marker='o', color='#C44E52')
# Set title and axis labels for precision plot
ax1.set_title('Precision Comparison')
ax1.set_xlabel('Plane')
ax1.set_ylabel('Precision')
# Add a legend to the precision plot
ax1.legend()

# Plot Recall
# Plot recall for RG_PCL, RANSAC_PCL, and RANSAC_O3D with markers for each plane
ax2.plot(planes, recall_rg, label='RG_PCL', marker='o', color='#4C72B0')
ax2.plot(planes, recall_ransac_pcl, label='RANSAC_PCL', marker='o', color='#55A868')
ax2.plot(planes, recall_ransac_o3d, label='RANSAC_O3D', marker='o', color='#C44E52')
# Set title and axis labels for recall plot
ax2.set_title('Recall Comparison')
ax2.set_xlabel('Plane')
ax2.set_ylabel('Recall')
# Add a legend to the recall plot
ax2.legend()

# Adjust layout to ensure there is no overlap and plots are spaced properly
plt.tight_layout()
# Display the plots
plt.show()
