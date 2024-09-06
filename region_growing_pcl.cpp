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

    // 记录程序开始时间
    auto start = std::chrono::high_resolution_clock::now();

    // 记录程序开始时间 - 读取点云文件
    auto start_reading = std::chrono::high_resolution_clock::now();

    // 使用 PointXYZRGB 读取包含颜色信息的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // 记录读取点云结束时间
    auto end_reading = std::chrono::high_resolution_clock::now();

    // 打印原始点云的点数
    std::cout << "Original cloud size: " << cloud->points.size() << " points" << std::endl;

    // 将字符串转换为浮点数
    float leaf_size = std::stof(argv[2]);

    // 记录开始时间 - 体素下采样
    auto start_downsampling = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (leaf_size > 0) {
        // 体素下采样
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);  // 设置体素大小
        sor.filter(*cloud_filtered);

        std::cout << "Filtered cloud size: " << cloud_filtered->points.size() << " points" << std::endl;
    } else {
        // 如果 leaf_size 为 0，则不进行下采样
        cloud_filtered = cloud;
        std::cout << "Leaf size is 0, no downsampling applied." << std::endl;
    }

    // 记录下采样结束时间
    auto end_downsampling = std::chrono::high_resolution_clock::now();

    // 记录开始时间 - 计算法向量
    auto start_normals = std::chrono::high_resolution_clock::now();

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_filtered);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // 记录计算法向量结束时间
    auto end_normals = std::chrono::high_resolution_clock::now();

    // 记录开始时间 - 区域生长算法
    auto start_region_growing = std::chrono::high_resolution_clock::now();
    
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(300);  // 设置最小的簇大小
    reg.setMaxClusterSize(1000000);  // 设置最大的簇大小
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);  // 邻域大小
    reg.setInputCloud(cloud_filtered);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // 设置平滑度阈值
    reg.setCurvatureThreshold(1.0);  // 设置曲率阈值

    // 提取聚类
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // 记录区域生长算法结束时间
    auto end_region_growing = std::chrono::high_resolution_clock::now();

    // 记录开始时间 - 改变聚类颜色并存储
    auto start_color_and_store = std::chrono::high_resolution_clock::now();

    // 初始化随机数生成器
    std::srand(static_cast<unsigned int>(std::time(0)));

    // 创建一个新的点云来存储所有聚类
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j = 0;
    for (const auto& indices : clusters) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : indices.indices) {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        }

        // 生成随机颜色
        int r = std::rand() % 256;
        int g = std::rand() % 256;
        int b = std::rand() % 256;

        // 更新当前聚类中每个点的颜色
        for (auto& point : cloud_cluster->points) {
            point.r = r;
            point.g = g;
            point.b = b;
        }

        // 将当前聚类添加到所有聚类的点云中
        *cloud_all_clusters += *cloud_cluster;

        j++;
    }

    // 记录改变聚类颜色并存储结束时间
    auto end_color_and_store = std::chrono::high_resolution_clock::now();

    // 记录开始时间 - 保存新PCD文件
    auto start_saving_pcd = std::chrono::high_resolution_clock::now();

    // 生成新的文件名，在原文件名基础上添加后缀
    std::string input_filename = argv[1];
    std::string output_filename = input_filename.substr(0, input_filename.find_last_of(".")) + "_rg_pcl.pcd";

    // 保存修改后的点云到新的文件
    pcl::io::savePCDFileASCII(output_filename, *cloud_all_clusters);

    // 记录保存新PCD文件结束时间
    auto end_saving_pcd = std::chrono::high_resolution_clock::now();

    // 记录程序结束时间
    auto end = std::chrono::high_resolution_clock::now();
    // 打印保存后的信息
    std::cout << "Saved point cloud to " << output_filename << " with " << cloud_all_clusters->points.size() << " points." 
    << " and " << clusters.size() << " clusters " << std::endl;

    // 记录每个阶段的时间并打印
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
