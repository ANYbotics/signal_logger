/*!
 * @file     SignalLoggerRos.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 23, 2016
 * @brief    Implementation of a ROS signal logger. Provides necessary ROS communication.
 *           Extends std logger with publishing and bag storing functionality.
 */

#pragma once

#ifdef ROS2_BUILD
// signal logger
#include "signal_logger_core/typedefs.hpp"
#include "signal_logger_ros/LogElementRos.hpp"
#include "signal_logger_std/SignalLoggerStd.hpp"

// msgs
#include <signal_logger_msgs/srv/edit_logger_script.hpp>
#include <signal_logger_msgs/srv/get_logger_configuration.hpp>
#include <signal_logger_msgs/srv/get_logger_element.hpp>
#include <signal_logger_msgs/srv/save_logger_data.hpp>
#include <signal_logger_msgs/srv/set_logger_element.hpp>

// rosbag
#include <rosbag2_cpp/writer.hpp>

// ros services
#include <std_srvs/srv/trigger.hpp>

namespace signal_logger_ros {

class SignalLoggerRos : public signal_logger_std::SignalLoggerStd {
 public:
  /** Constructor
   *  @param node             pointer to the ros node
   */
  explicit SignalLoggerRos(rclcpp::Node::SharedPtr node);

  //! Destructor
  ~SignalLoggerRos() override = default;

  /** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
   * @tparam ValueType_       Data type of the logger element
   * @param  var              Pointer to log variable
   * @param  name             name of the log variable
   * @param  group            logger group the variable belongs to
   * @param  unit             unit of the log variable
   * @param  divider          divider is defining the update frequency of the logger element (ctrl_freq/divider)
   * @param  action           log action of the log variable
   * @param  bufferSize       size of the buffer storing log elements
   * @param  bufferType       determines the buffer type
   */
  template <typename ValueType_>
  void add(const ValueType_* const var, const std::string& name,
           const std::string& group = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
           const std::string& unit = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
           const std::size_t divider = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
           const signal_logger::LogElementAction action = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
           const std::size_t bufferSize = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
           const signal_logger::BufferType bufferType = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE) {
    std::string elementName = options_.loggerPrefix_ + "/" + group + "/" + name;
    elementName.erase(std::unique(elementName.begin(), elementName.end(), signal_logger::both_slashes()), elementName.end());
    {
      // Lock the logger (blocking!)
      boost::unique_lock<boost::shared_mutex> addLoggerLock(loggerMutex_);
      logElementsToAdd_[elementName].reset(new LogElementRos<ValueType_>(var, bufferType, bufferSize, elementName, unit, divider, action,
                                                                         &textStream_, &binaryStream_, node_, bagWriter_));
    }
  }

  //! Shutdown ros communication
  bool cleanup() override;

  /** Returns the current time
   * @return current time
   */
  signal_logger::TimestampPair getCurrentTime() override;

  //! Save all the buffered data into a log file
  bool workerSaveData(const std::string& logFileName, const std::string& pathWithPrefix,
                      const signal_logger::LogFileTypeSet& logfileTypes) override;

  /** Get current logger configuration
   *  @param  req empty request
   *  @param  res logger_namespace, script_filepath, log_element_names, collect_frequency
   */
  void getLoggerConfiguration(signal_logger_msgs::srv::GetLoggerConfiguration::Request::SharedPtr req,
                              signal_logger_msgs::srv::GetLoggerConfiguration::Response::SharedPtr res);

  /** Get logger element
   *  @param  req element name
   *  @param  res logger element
   */
  void getLoggerElement(signal_logger_msgs::srv::GetLoggerElement::Request::SharedPtr req,
                        signal_logger_msgs::srv::GetLoggerElement::Response::SharedPtr res);

  /** Set logger element
   *  @param  req log element
   *  @param  res success status
   */
  void setLoggerElement(signal_logger_msgs::srv::SetLoggerElement::Request::SharedPtr req,
                        signal_logger_msgs::srv::SetLoggerElement::Response::SharedPtr res);

  /** Start Logger
   *  @param  req empty request
   *  @param  res success status
   */
  void startLogger(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

  /** Stop Logger
   *  @param  req empty request
   *  @param  res success status
   */
  void stopLogger(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

  /** Save Logger data
   *  @param  req empty request
   *  @param  res success status
   */
  void saveLoggerData(signal_logger_msgs::srv::SaveLoggerData::Request::SharedPtr req,
                      signal_logger_msgs::srv::SaveLoggerData::Response::SharedPtr res);

  /** Is logger running
   *  @param  req empty request
   *  @param  res is logger running flag
   */
  void isLoggerRunning(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

  /** Load logger script
   *  @param  req file path
   *  @param  res success status
   */
  void loadLoggerScript(signal_logger_msgs::srv::EditLoggerScript::Request::SharedPtr req,
                        signal_logger_msgs::srv::EditLoggerScript::Response::SharedPtr res);

  /** Save logger script
   *  @param  req file path
   *  @param  res success status
   */
  void saveLoggerScript(signal_logger_msgs::srv::EditLoggerScript::Request::SharedPtr req,
                        signal_logger_msgs::srv::EditLoggerScript::Response::SharedPtr res);

  /** Write log element to msg
   *  @param  name log element name
   *  @param  msg  log element message
   *  @return true iff successful
   */
  bool logElementtoMsg(const std::string& name, signal_logger_msgs::msg::LogElement& msg);

  /** Write msg to log element
   *  @param  msg  log element message (contains name of log element to write)
   *  @return true iff successful
   */
  bool msgToLogElement(const signal_logger_msgs::msg::LogElement& msg);

 private:
  //! ROS nodehandle
  rclcpp::Node::SharedPtr node_;
  //! Rosbag writer object
  rosbag2_cpp::Writer bagWriter_;
  //! Get logger configuration service
  rclcpp::Service<signal_logger_msgs::srv::GetLoggerConfiguration>::SharedPtr getLoggerConfigurationService_;
  //! Get logger element service
  rclcpp::Service<signal_logger_msgs::srv::GetLoggerElement>::SharedPtr getLoggerElementService_;
  //! Set logger element service
  rclcpp::Service<signal_logger_msgs::srv::SetLoggerElement>::SharedPtr setLoggerElementService_;
  //! Start logger service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startLoggerService_;
  //! Stop logger service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopLoggerService_;
  //! Save logger data service
  rclcpp::Service<signal_logger_msgs::srv::SaveLoggerData>::SharedPtr saveLoggerDataService_;
  //! Load logger script service
  rclcpp::Service<signal_logger_msgs::srv::EditLoggerScript>::SharedPtr loadLoggerScriptService_;
  //! Save logger script service
  rclcpp::Service<signal_logger_msgs::srv::EditLoggerScript>::SharedPtr saveLoggerScriptService_;
  //! Is logger running service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr isLoggerRunningService_;
};

}  // namespace signal_logger_ros
#endif /* ROS2_BUILD */
