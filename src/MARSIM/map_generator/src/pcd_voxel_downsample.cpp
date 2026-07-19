#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: pcd_voxel_downsample <input.pcd> <output.pcd> "
                 "<leaf_size_m>\n";
    return 1;
  }

  const std::string input_path(argv[1]);
  const std::string output_path(argv[2]);
  double leaf_size = 0.0;
  try {
    leaf_size = std::stod(argv[3]);
  } catch (const std::exception &) {
    std::cerr << "Invalid leaf size: " << argv[3] << '\n';
    return 1;
  }

  if (leaf_size <= 0.0) {
    std::cerr << "Leaf size must be positive.\n";
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, input_cloud) != 0) {
    std::cerr << "Failed to load: " << input_path << '\n';
    return 1;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(input_cloud.makeShared());
  voxel_filter.setLeafSize(static_cast<float>(leaf_size),
                           static_cast<float>(leaf_size),
                           static_cast<float>(leaf_size));

  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  voxel_filter.filter(output_cloud);
  if (pcl::io::savePCDFileBinaryCompressed(output_path, output_cloud) != 0) {
    std::cerr << "Failed to write: " << output_path << '\n';
    return 1;
  }

  std::cout << "Voxel downsampled " << input_cloud.size() << " -> "
            << output_cloud.size() << " points (leaf=" << leaf_size << " m)\n";
  return 0;
}
