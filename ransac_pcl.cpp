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

    // 初始化随机数种子
    std::srand(static_cast<unsigned int>(std::time(0))); // 使用当前时间作为种子

    // 记录程序开始时间
    auto start = std::chrono::high_resolution_clock::now();

    // 记录读取 PCD 文件的开始时间
    auto read_start = std::chrono::high_resolution_clock::now();

    // 读取点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file \n");
        return -1;
    }

    // 记录读取 PCD 文件的结束时间
    auto read_end = std::chrono::high_resolution_clock::now();

    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

    // 创建平面模型的分割对象
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

    // 记录 RANSAC 处理的开始时间
    auto ransac_start = std::chrono::high_resolution_clock::now();

    while (cloud->points.size() > 1500) {
        // 拟合平面
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // std::cout << "Plane " << i << " coefficients: " << coefficients->values[0] << " "
        //           << coefficients->values[1] << " "
        //           << coefficients->values[2] << " "
        //           << coefficients->values[3] << std::endl;

        // 提取平面内的点
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // 为平面点云分配随机颜色
        int r = std::rand() % 256;
        int g = std::rand() % 256;
        int b = std::rand() % 256;
        for (auto& point : cloud_plane->points) {
            point.r = static_cast<uint8_t>(r);
            point.g = static_cast<uint8_t>(g);
            point.b = static_cast<uint8_t>(b);
        }

        // std::cout << "Point cloud representing the planar component: " << cloud_plane->points.size() << " points." << std::endl;

        // 将当前平面点云添加到总的点云中
        *cloud_planes += *cloud_plane;

        // 移除平面内的点，继续寻找下一个平面
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud = *cloud_f;

        i++;
    }

    // 记录 RANSAC 处理的结束时间
    auto ransac_end = std::chrono::high_resolution_clock::now();

    // 基于源文件名生成新的 PCD 文件名
    std::string input_filename = argv[1];
    std::string output_filename = input_filename.substr(0, input_filename.find_last_of(".")) + "_ransac_pcl.pcd";

    // 记录保存 PCD 文件的开始时间
    auto save_start = std::chrono::high_resolution_clock::now();

    // 保存所有平面到一个 PCD 文件
    pcl::io::savePCDFileASCII(output_filename, *cloud_planes);

    // 记录保存 PCD 文件的结束时间
    auto save_end = std::chrono::high_resolution_clock::now();

    // 记录程序结束时间
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Saved point cloud to " << output_filename << " with " << cloud_planes->points.size() << " points." 
    << " and " << i << " clusters " << std::endl;


    // 计算并打印所有时间
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
