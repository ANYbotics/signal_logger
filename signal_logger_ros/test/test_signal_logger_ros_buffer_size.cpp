/**
 * @authors     Zichong Li 
 * @affiliation ANYbotics
 */

 #include <gtest/gtest.h>

 #include <signal_logger_ros/SignalLoggerRos.hpp>
 
 #ifndef ROS2_BUILD
 
 class TestSignalLoggerRosBufferSize : public ::testing::Test {
  protected:
   void SetUp() override {
    logger_ = std::make_shared<signal_logger_ros::SignalLoggerRos>(node);
    signal_logger::SignalLoggerOptions options{updateFrequency_, maxLoggingTime_, "test_script.yaml", "/log"};
    logger_->initLogger(options);
   }
 
   void TearDown() override {
    logger_->cleanup();
   }
 
   ros::NodeHandle nh{"~"};
   ros::NodeHandle* node = &nh;
 
 #else  /* ROS2_BUILD */
 
 class TestSignalLoggerRosBufferSize : public ::testing::Test {
  protected:
   void SetUp() override {
    node = rclcpp::Node::make_shared("signal_logger_ros_test");
 
    logger_ = std::make_shared<signal_logger_ros::SignalLoggerRos>(node);
    signal_logger::SignalLoggerOptions options{updateFrequency_, maxLoggingTime_, "test_script.yaml", "/log"};
    logger_->initLogger(options);
   }
 
   void TearDown() override {
    logger_->cleanup();
   }
 
   rclcpp::Node::SharedPtr node;
   #endif /* ROS2_BUILD */
   std::shared_ptr<signal_logger_ros::SignalLoggerRos> logger_;
   const unsigned int updateFrequency_{100};
   const double maxLoggingTime_{10.0};
   const float customBufferSize_{42};
   const std::string defaultLogElementName_{"logVar1"};
   const std::string defaultGroupName_{"test"};
   const std::string fullElementName_ = "/log/" + defaultGroupName_ + "/" + defaultLogElementName_;
   float logVar1_{0.0};
  };
 
 TEST_F(TestSignalLoggerRosBufferSize, checkDefaultBufferSize) {  // NOLINT
   // Check if the default buffer size is set correctly.
   logger_->add(&logVar1_, defaultLogElementName_, defaultGroupName_);
   EXPECT_TRUE(logger_->updateLogger());
   EXPECT_EQ(logger_->getElement(fullElementName_).getBuffer().getBufferSize(), updateFrequency_* maxLoggingTime_);
 }
 
 TEST_F(TestSignalLoggerRosBufferSize, checkCustomBufferSize) {  // NOLINT
  // Check if the custom buffer size is set correctly.
  logger_->add(&logVar1_, defaultLogElementName_, defaultGroupName_, "[-]", 1, signal_logger::LogElementAction::SAVE, signal_logger::BufferType::LOOPING, customBufferSize_);
  EXPECT_TRUE(logger_->updateLogger());
  EXPECT_EQ(logger_->getElement(fullElementName_).getBuffer().getBufferSize(), customBufferSize_);
}