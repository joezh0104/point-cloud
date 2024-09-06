#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string>
#include <chrono>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <leaf_size>" << std::endl;
        return -1;
    }

    // Record the start time of the program
    auto start = std::chrono::high_resolution_clock::now();

    // Record the start time - reading the point cloud file
    auto start_reading = std::chrono::high_resolution_clock::now();

    // Use PointXYZRGB to read the point cloud with color information
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // Record the end time of reading the point cloud
    auto end_reading = std::chrono::high_resolution_clock::now();

    // Print the number of points in the original point cloud
    std::cout << "Original cloud size: " << cloud->points.size() << " points" << std::endl;

    // Convert the string to a floating-point number
    float leaf_size = std::stof(argv[2]);

    // Record the start time - voxel downsampling
    auto start_downsampling = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (leaf_size > 0) {
        // Voxel downsampling
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);  // Set voxel size
        sor.filter(*cloud_filtered);

        std::cout << "Filtered cloud size: " << cloud_filtered->points.size() << " points" << std::endl;
    } else {
        // If leaf_size is 0, no downsampling is applied
        cloud_filtered = cloud;
        std::cout << "Leaf size is 0, no downsampling applied." << std::endl;
    }

    // Record the end time of downsampling
    auto end_downsampling = std::chrono::high_resolution_clock::now();

    // Record the start time - normal vector calculation
    auto start_normals = std::chrono::high_resolution_clock::now();

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_filtered);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // Record the end time of normal vector calculation
    auto end_normals = std::chrono::high_resolution_clock::now();

    // Record the start time - region growing algorithm
    auto start_region_growing = std::chrono::high_resolution_clock::now();
    
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(300);  // Set minimum cluster size
    reg.setMaxClusterSize(1000000);  // Set maximum cluster size
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);  // Neighborhood size
    reg.setInputCloud(cloud_filtered);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // Set smoothness threshold
    reg.setCurvatureThreshold(1.0);  // Set curvature threshold

    // Extract clusters
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // Record the end time of the region growing algorithm
    auto end_region_growing = std::chrono::high_resolution_clock::now();

    // Record the start time - coloring clusters and storing
    auto start_color_and_store = std::chrono::high_resolution_clock::now();

    // Initialize random number generator
    std::srand(static_cast<unsigned int>(std::time(0)));

    // Create a new point cloud to store all clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j = 0;
    for (const auto& indices : clusters) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : indices.indices) {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        }

        // Generate random colors
        int r = std::rand() % 256;
        int g = std::rand() % 256;
        int b = std::rand() % 256;

        // Update the color of each point in the current cluster
        for (auto& point : cloud_cluster->points) {
            point.r = r;
            point.g = g;
            point.b = b;
        }

        // Add the current cluster to the point cloud containing all clusters
        *cloud_all_clusters += *cloud_cluster;

        j++;
    }

    // Record the end time of coloring clusters and storing
    auto end_color_and_store = std::chrono::high_resolution_clock::now();

    // Record the start time - saving the new PCD file
    auto start_saving_pcd = std::chrono::high_resolution_clock::now();

    // Generate a new filename by adding a suffix to the original file name
    std::string input_filename = argv[1];
    std::string output_filename = input_filename.substr(0, input_filename.find_last_of(".")) + "_rg_pcl.pcd";

    // Save the modified point cloud to the new file
    pcl::io::savePCDFileASCII(output_filename, *cloud_all_clusters);

    // Record the end time of saving the new PCD file
    auto end_saving_pcd = std::chrono::high_resolution_clock::now();

    // Record the end time of the program
    auto end = std::chrono::high_resolution_clock::now();
    // Print the information after saving
    std::cout << "Saved point cloud to " << output_filename << " with " << cloud_all_clusters->points.size() << " points." 
    << " and " << clusters.size() << " clusters " << std::endl;

    // Record the time taken by each stage and print
    std::chrono::duration<double> duration_reading = end_reading - start_reading;
    std::chrono::duration<double> duration_downsampling = end_downsampling - start_downsampling;
    std::chrono::duration<double> duration_normals = end_normals - start_normals;
    std::chrono::duration<double> duration_region_growing = end_region_growing - start_region_growing;
    std::chrono::duration<double> duration_color_and_store = end_color_and_store - start_color_and_store;
    std::chrono::duration<double> duration_saving_pcd = end_saving_pcd - start_saving_pcd;
    std::chrono::duration<double> total_duration = end - start;

    std::cout << "Reading time: " << duration_reading.count() << " seconds" << std::endl;
    std::cout << "Downsampling time: " << duration_downsampling.count() << " seconds" << std::endl;
    std::cout << "Normal estimation time: " << duration_normals.count() << " seconds" << std::endl;
    std::cout << "Region growing time: " << duration_region_growing.count() << " seconds" << std::endl;
    std::cout << "Coloring and storing clusters time: " << duration_color_and_store.count() << " seconds" << std::endl;
    std::cout << "Saving PCD file time: " << duration_saving_pcd.count() << " seconds" << std::endl;
    std::cout << "Total execution time: " << total_duration.count() << " seconds" << std::endl;

    return 0;
}
