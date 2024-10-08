#include <gtest/gtest.h>

#include <signal_logger_ros/SignalLoggerRos.hpp>

#ifndef ROS2_BUILD

class SignalLoggerRos : public ::testing::Test {
 protected:
  void SetUp() override {}

  void TearDown() override {}

  ros::NodeHandle nh{"~"};
  ros::NodeHandle* node = &nh;
};

#else  /* ROS2_BUILD */

class SignalLoggerRos : public ::testing::Test {
 protected:
  void SetUp() override {
    node = rclcpp::Node::make_shared("signal_logger_ros_test");

    spinThread = std::thread([this]() {
      executor.add_node(node);
      executor.spin();
      executor.remove_node(node);
    });
  }

  void TearDown() override {
    executor.cancel();
    spinThread.join();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinThread;
};
#endif /* ROS2_BUILD */

TEST_F(SignalLoggerRos, testSignalLoggerRos) {  // NOLINT
  // Variables to log
  int myInt = 42;
  Eigen::Vector3d myVector(1., 2., 3.);

  signal_logger_ros::SignalLoggerRos logger(node);
  signal_logger::SignalLoggerOptions options;
  options.maxLoggingTime_ = 10.;
  options.updateFrequency_ = 100;
  options.loggerPrefix_ = "";
  logger.initLogger(options);
  logger.add(&myInt, "my_int");
  logger.add(&myVector, "my_vector");
  logger.updateLogger();
  logger.setName("testSignalLoggerRos");
  logger.SignalLoggerBase::startLogger(false);

  logger.collectLoggerData();
  myVector << 4., 5., 6.;
  myInt = 43;
  logger.collectLoggerData();

  logger.stopAndSaveLoggerData({signal_logger::LogFileType::BINARY, signal_logger::LogFileType::BAG});
  logger.cleanup();
}
