#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "static_pcd_publisher");
  ros::NodeHandle private_nh("~");

  std::string file_name;
  std::string frame_id;
  std::string topic_name;
  private_nh.param<std::string>("file_name", file_name, "");
  private_nh.param<std::string>("frame_id", frame_id, "world");
  private_nh.param<std::string>("topic_name", topic_name,
                                "/debug/generated_pcd_map");
  if (file_name.empty()) {
    ROS_ERROR("~file_name is required");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) != 0) {
    ROS_ERROR_STREAM("Failed to load PCD: " << file_name);
    return 1;
  }

  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;
  message.header.stamp = ros::Time::now();

  ros::Publisher publisher = private_nh.advertise<sensor_msgs::PointCloud2>(
      topic_name, 1, true);
  publisher.publish(message);
  ROS_INFO_STREAM("Published static RViz map once: " << cloud.size()
                  << " points on " << topic_name);

  // A live latched publisher supplies this one message to RViz instances that
  // subscribe later, without continuously republishing the large cloud.
  ros::spin();
  return 0;
}
