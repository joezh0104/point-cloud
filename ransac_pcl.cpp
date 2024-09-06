#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <random>
#include <ctime>
#include <chrono>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd>" << std::endl;
        return -1;
    }

    // Initialize random number seed
    std::srand(static_cast<unsigned int>(std::time(0))); // Use the current time as the seed

    // Record the start time of the program
    auto start = std::chrono::high_resolution_clock::now();

    // Record the start time of reading the PCD file
    auto read_start = std::chrono::high_resolution_clock::now();

    // Read the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file \n");
        return -1;
    }

    // Record the end time of reading the PCD file
    auto read_end = std::chrono::high_resolution_clock::now();

    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

    // Create a segmentation object for the planar model
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.2);

    int i = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planes(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Record the start time of RANSAC processing
    auto ransac_start = std::chrono::high_resolution_clock::now();

    while (cloud->points.size() > 1500) {
        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the points belonging to the plane
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // Assign random colors to the points of the plane
        int r = std::rand() % 256;
        int g = std::rand() % 256;
        int b = std::rand() % 256;
        for (auto& point : cloud_plane->points) {
            point.r = static_cast<uint8_t>(r);
            point.g = static_cast<uint8_t>(g);
            point.b = static_cast<uint8_t>(b);
        }

        // Add the current plane point cloud to the total point cloud
        *cloud_planes += *cloud_plane;

        // Remove the points belonging to the plane and continue searching for the next plane
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud = *cloud_f;

        i++;
    }

    // Record the end time of RANSAC processing
    auto ransac_end = std::chrono::high_resolution_clock::now();

    // Generate a new PCD file name based on the input file name
    std::string input_filename = argv[1];
    std::string output_filename = input_filename.substr(0, input_filename.find_last_of(".")) + "_ransac_pcl.pcd";

    // Record the start time of saving the PCD file
    auto save_start = std::chrono::high_resolution_clock::now();

    // Save all the planes to a PCD file
    pcl::io::savePCDFileASCII(output_filename, *cloud_planes);

    // Record the end time of saving the PCD file
    auto save_end = std::chrono::high_resolution_clock::now();

    // Record the end time of the program
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Saved point cloud to " << output_filename << " with " << cloud_planes->points.size() << " points." 
    << " and " << i << " clusters " << std::endl;


    // Calculate and print all timings
    std::chrono::duration<double> read_duration = read_end - read_start;
    std::chrono::duration<double> ransac_duration = ransac_end - ransac_start;
    std::chrono::duration<double> save_duration = save_end - save_start;
    std::chrono::duration<double> total_duration = end - start;

    std::cout << "Time taken to read PCD file: " << read_duration.count() << " seconds" << std::endl;
    std::cout << "Time taken for RANSAC processing: " << ransac_duration.count() << " seconds" << std::endl;
    std::cout << "Time taken to save PCD file: " << save_duration.count() << " seconds" << std::endl;
    std::cout << "Total execution time: " << total_duration.count() << " seconds" << std::endl;

    return 0;
}
