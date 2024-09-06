import open3d as o3d
import numpy as np
import os
import time  

# Load the point cloud data
denoised_cloud = o3d.io.read_point_cloud("cube_point_cloud.pcd")
o3d.visualization.draw_geometries([denoised_cloud])

# Print the number of points in the original data
original_point_count = len(denoised_cloud.points)
print(f"Number of points in the original data: {original_point_count}")

# Initialize lists to store the plane point clouds
denoised_plane_clouds = []
plane_models = []

# Record the start time of the algorithm
start_time = time.time()

# Perform iterative plane segmentation
while True:
    # Perform plane segmentation
    current_plane_model, inliers = denoised_cloud.segment_plane(distance_threshold=0.05, ransac_n=3,
                                                                num_iterations=1000)

    # If no more planes are found, exit the loop
    if len(inliers) < 1500:  # Adjust the threshold based on the actual scenario
        break

    # Extract the point cloud corresponding to the detected plane
    denoised_plane_cloud = denoised_cloud.select_by_index(inliers)

    # Assign a random color to the plane point cloud
    random_color = np.random.rand(3)  # Generate random color
    denoised_plane_cloud.paint_uniform_color(random_color)

    # Store the plane point cloud
    denoised_plane_clouds.append(denoised_plane_cloud)
    plane_models.append(current_plane_model)

    # Remove the segmented plane from the point cloud
    denoised_cloud = denoised_cloud.select_by_index(inliers, invert=True)

# Record the end time of the algorithm
end_time = time.time()

# Print the color of each plane
for i, denoised_plane_cloud in enumerate(denoised_plane_clouds):
    color_array = np.asarray(denoised_plane_cloud.colors)[0]
    print(f"Random color of plane {i}: {color_array}")

# Visualize the result
o3d.visualization.draw_geometries(denoised_plane_clouds)

# Print the final number of points
final_point_count = sum(len(plane_cloud.points) for plane_cloud in denoised_plane_clouds)
print(f"Final number of points: {final_point_count}")

# Print the algorithm execution time
elapsed_time = end_time - start_time
print(f"Algorithm execution time: {elapsed_time:.2f} seconds")

# Save each plane point cloud to the desktop
desktop_path = os.path.join(os.path.join(os.environ['USERPROFILE']), 'Desktop')
for i, plane_cloud in enumerate(denoised_plane_clouds):
    file_path = os.path.join(desktop_path, f"plane_{i}.pcd")
    o3d.io.write_point_cloud(file_path, plane_cloud)
    print(f"Plane {i} has been saved to {file_path}")

# Merge all plane point clouds into a single combined point cloud
combined_cloud = denoised_plane_clouds[0]
for plane_cloud in denoised_plane_clouds[1:]:
    combined_cloud += plane_cloud

# Save the combined point cloud to the desktop
combined_file_path = os.path.join(desktop_path, "combined_planes.pcd")
o3d.io.write_point_cloud(combined_file_path, combined_cloud)
print(f"Combined point cloud has been saved to {combined_file_path}")
