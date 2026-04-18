#ifndef ROSARIA2_AR_TIME_TO_ROS_TIME_HPP
#define ROSARIA2_AR_TIME_TO_ROS_TIME_HPP

#include <rclcpp/rclcpp.hpp>
#include <Aria/ariaUtil.h>

inline rclcpp::Time convertArTimeToROS(const ArTime& t, rclcpp::Clock::SharedPtr clock)
{
  // ARIA/ARNL times are in reference to an arbitrary starting time, not OS
  // clock, so find the time elapsed between now and t
  // to adjust the time stamp in ROS time vs. now accordingly.
  ArTime arianow;
  const double dtsec = static_cast<double>(t.mSecSince(arianow)) / 1000.0;
  // t.mSecSince(arianow) is negative if t is in the past.
  // now.seconds() + dtsec will give a time in the past.
  rclcpp::Time now = clock->now();
  return rclcpp::Time(
    static_cast<int64_t>((now.seconds() + dtsec) * 1e9),
    RCL_ROS_TIME);
}

#endif // ROSARIA2_AR_TIME_TO_ROS_TIME_HPP
