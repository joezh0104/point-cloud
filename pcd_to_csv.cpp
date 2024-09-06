#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // 检查输入参数
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd>" << std::endl;
        return -1;
    }

    // 读取 PCD 文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    // 提取源文件的基本名称（去掉路径和扩展名）
    std::string input_filename = argv[1];
    size_t last_dot = input_filename.find_last_of(".");
    size_t last_slash = input_filename.find_last_of("/\\");
    std::string base_filename = input_filename.substr((last_slash == std::string::npos ? 0 : last_slash + 1), 
                                                      (last_dot == std::string::npos ? input_filename.size() : last_dot - (last_slash == std::string::npos ? 0 : last_slash + 1)));

    // 基于源文件名生成新的 CSV 文件名
    std::string output_filename = base_filename + ".csv";

    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error opening output file." << std::endl;
        return -1;
    }

    // 写入 CSV 文件头
    outfile << "x,y,z" << std::endl;

    // 将点云数据写入 CSV 文件
    for (const auto& point : cloud->points) {
        outfile << point.x << "," << point.y << "," << point.z << std::endl;
    }

    outfile.close();
    std::cout << "点云数据已成功转换为 CSV 文件: " << output_filename << std::endl;
    return 0;
}
