#include "rosaria2/laser_publisher.hpp"

#include <Aria/Aria.h>
#include <Aria/ariaUtil.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>

LaserPublisher::LaserPublisher(ArLaser *laser, rclcpp::Node::SharedPtr node,
                               bool broadcast_transform,
                               const std::string &tf_frame,
                               const std::string &parent_tf_frame,
                               const std::string &global_tf_frame)
    : laser_readings_cb_(this, &LaserPublisher::readingsCB), node_(node),
      laser_(laser), tf_name_(tf_frame), parent_tf_name_(parent_tf_frame),
      global_tf_name_(global_tf_frame), broadcast_tf_(broadcast_transform) {
  assert(laser_);
  laser_->lockDevice();
  laser_->addReadingCB(&laser_readings_cb_);
  laser_->unlockDevice();

  // Build topic names from laser name (strip dots)
  std::string laser_name(laser_->getName());
  laser_name.erase(std::remove(laser_name.begin(), laser_name.end(), '.'),
                   laser_name.end());

  std::string laserscan_topic = laser_name + "_laserscan";
  std::string pointcloud_topic = laser_name + "_pointcloud";

  laserscan_pub_ =
      node_->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_topic, 20);
  pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>(
      pointcloud_topic, 50);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Build the static transform for the laser
  laser_tf_.header.frame_id = parent_tf_name_;
  laser_tf_.child_frame_id = tf_name_;

  tf2::Quaternion q;
  if (laser_->hasSensorPosition()) {
    laser_tf_.transform.translation.x = laser_->getSensorPositionX() / 1000.0;
    laser_tf_.transform.translation.y = laser_->getSensorPositionY() / 1000.0;
    laser_tf_.transform.translation.z = laser_->getSensorPositionZ() / 1000.0;
    q.setRPY(0, 0, ArMath::degToRad(laser_->getSensorPositionTh()));
  } else {
    laser_tf_.transform.translation.x = 0.0;
    laser_tf_.transform.translation.y = 0.0;
    laser_tf_.transform.translation.z = 0.0;
    q.setRPY(0, 0, 0);
  }
  laser_tf_.transform.rotation = tf2::toMsg(q);

  // Pre-fill laser scan static fields
  laserscan_.header.frame_id = tf_name_;
  laserscan_.angle_min = ArMath::degToRad(laser_->getStartDegrees());
  laserscan_.angle_max = ArMath::degToRad(laser_->getEndDegrees());
  laserscan_.range_min = 0.0f;
  laserscan_.range_max = laser_->getMaxRange() / 1000.0f;

  // Get angle_increment
  laserscan_.angle_increment = 0.0f;
  if (laser_->canSetIncrement()) {
    laserscan_.angle_increment = laser_->getIncrement();
  } else if (laser_->getIncrementChoice() != NULL) {
    laserscan_.angle_increment = laser_->getIncrementChoiceDouble();
  }
  assert(laserscan_.angle_increment > 0);
  laserscan_.angle_increment *= static_cast<float>(M_PI / 180.0);

  pointcloud_.header.frame_id = global_tf_name_;
}

LaserPublisher::~LaserPublisher() {
  laser_->lockDevice();
  laser_->remReadingCB(&laser_readings_cb_);
  laser_->unlockDevice();
}

void LaserPublisher::readingsCB() {
  assert(laser_);
  laser_->lockDevice();
  publishLaserScan();
  publishPointCloud();
  laser_->unlockDevice();

  if (broadcast_tf_) {
    laser_tf_.header.stamp =
        convertArTimeToROS(laser_->getLastReadingTime(), node_->get_clock());
    tf_broadcaster_->sendTransform(laser_tf_);
  }
}

void LaserPublisher::publishLaserScan() {
  laserscan_.header.stamp =
      convertArTimeToROS(laser_->getLastReadingTime(), node_->get_clock());
  const std::list<ArSensorReading *> *readings = laser_->getRawReadings();
  assert(readings);
  laserscan_.ranges.resize(readings->size());

  size_t n = 0;
  if (laser_->getFlipped()) {
    for (auto r = readings->rbegin(); r != readings->rend(); ++r) {
      assert(*r);
      if ((*r)->getIgnoreThisReading()) {
        laserscan_.ranges[n] = -1.0f;
      } else {
        laserscan_.ranges[n] = (*r)->getRange() / 1000.0f;
      }
      ++n;
    }
  } else {
    for (auto r = readings->begin(); r != readings->end(); ++r) {
      assert(*r);
      if ((*r)->getIgnoreThisReading()) {
        laserscan_.ranges[n] = -1.0f;
      } else {
        laserscan_.ranges[n] = (*r)->getRange() / 1000.0f;
      }
      ++n;
    }
  }
  laserscan_pub_->publish(laserscan_);
}

void LaserPublisher::publishPointCloud() {
  assert(laser_);
  pointcloud_.header.stamp =
      convertArTimeToROS(laser_->getLastReadingTime(), node_->get_clock());

  // AriaCoda returns buffer by value
  const std::list<ArPoseWithTime> p =
      laser_->getCurrentRangeBuffer().getBuffer();
  pointcloud_.points.resize(p.size());
  size_t n = 0;
  for (auto i = p.cbegin(); i != p.cend(); ++i) {
    pointcloud_.points[n].x = i->getX() / 1000.0f;
    pointcloud_.points[n].y = i->getY() / 1000.0f;
    pointcloud_.points[n].z =
        (laser_->hasSensorPosition() ? laser_->getSensorPositionZ() / 1000.0f
                                     : 0.0f);
    ++n;
  }
  pointcloud_pub_->publish(pointcloud_);
}
