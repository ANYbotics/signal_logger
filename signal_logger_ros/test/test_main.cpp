#ifndef ROS2_BUILD

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "signal_logger_ros_test");
  ros::start();
  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}

#else /* ROS2_BUILD */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}

#endif /* ROS2_BUILD */
