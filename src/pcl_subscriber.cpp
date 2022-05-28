#include <ros/ros.h>
// PCL specific includes

#include "pcl_conversion_df/ros_pointcloud2_to_pcl.hpp"


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  // --- pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);  ---
  pcl_df::fromROSMsg(*cloud_msg, pcl_cloud);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_2;
  pcl::fromROSMsg(*cloud_msg, pcl_cloud_2);

  ROS_INFO("%d %d", pcl_cloud.points.size(), pcl_cloud_2.points.size());
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 10, cloud_cb);
  // Spin
  ros::spin ();
}
