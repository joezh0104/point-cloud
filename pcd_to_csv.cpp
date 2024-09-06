#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // Check input arguments
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd>" << std::endl;
        return -1;
    }

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read the file\n");
        return (-1);
    }

    // Extract the base name of the input file (removing the path and extension)
    std::string input_filename = argv[1];
    size_t last_dot = input_filename.find_last_of(".");
    size_t last_slash = input_filename.find_last_of("/\\");
    std::string base_filename = input_filename.substr(
        (last_slash == std::string::npos ? 0 : last_slash + 1), 
        (last_dot == std::string::npos ? input_filename.size() : last_dot - (last_slash == std::string::npos ? 0 : last_slash + 1))
    );

    // Create a new CSV filename based on the input file name
    std::string output_filename = base_filename + ".csv";

    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error opening output file." << std::endl;
        return -1;
    }

    // Write the CSV header
    outfile << "x,y,z" << std::endl;

    // Write point cloud data to the CSV file
    for (const auto& point : cloud->points) {
        outfile << point.x << "," << point.y << "," << point.z << std::endl;
    }

    outfile.close();
    std::cout << "Point cloud data successfully converted to CSV file: " << output_filename << std::endl;
    return 0;
}
