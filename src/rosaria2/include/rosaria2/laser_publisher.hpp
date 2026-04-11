#ifndef ROSARIA2_LASER_PUBLISHER_HPP
#define ROSARIA2_LASER_PUBLISHER_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Aria/Aria.h>

#include "rosaria2/ar_time_to_ros_time.hpp"

class LaserPublisher {
public:
  LaserPublisher(ArLaser *laser, rclcpp::Node::SharedPtr node,
                 bool broadcast_transform = true,
                 const std::string &tf_frame = "laser",
                 const std::string &parent_tf_frame = "base_link",
                 const std::string &global_tf_frame = "odom");
  ~LaserPublisher();

protected:
  void readingsCB();
  void publishLaserScan();
  void publishPointCloud();

  ArFunctorC<LaserPublisher> laser_readings_cb_;
  rclcpp::Node::SharedPtr node_;
  ArLaser *laser_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub_;
  sensor_msgs::msg::LaserScan laserscan_;
  sensor_msgs::msg::PointCloud pointcloud_;
  std::string tf_name_;
  std::string parent_tf_name_;
  std::string global_tf_name_;
  geometry_msgs::msg::TransformStamped laser_tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool broadcast_tf_;
};

#endif // ROSARIA2_LASER_PUBLISHER_HPP
