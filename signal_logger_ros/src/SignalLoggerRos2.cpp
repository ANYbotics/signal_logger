/*!
 * @file     SignalLoggerRos.cpp
 * @author   Gabriel Hottiger
 * @date     Sep 23, 2016
 * @brief    Implementation of a ROS signal logger. Provides necessary ROS communication.
 *           Extends std logger with publishing and bag storing functionality.
 */

#ifdef ROS2_BUILD
#include <filesystem>
#include <functional>
#include "signal_logger_ros/SignalLoggerRos.hpp"

using namespace std::placeholders;

namespace signal_logger_ros {

SignalLoggerRos::SignalLoggerRos(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  getLoggerConfigurationService_ = node_->create_service<signal_logger_msgs::srv::GetLoggerConfiguration>(
      "silo_ros/get_logger_configuration", std::bind(&SignalLoggerRos::getLoggerConfigurationCb, this, _1, _2));
  getLoggerElementService_ = node_->create_service<signal_logger_msgs::srv::GetLoggerElement>(
      "silo_ros/get_logger_element", std::bind(&SignalLoggerRos::getLoggerElementCb, this, _1, _2));
  setLoggerElementService_ = node_->create_service<signal_logger_msgs::srv::SetLoggerElement>(
      "silo_ros/set_logger_element", std::bind(&SignalLoggerRos::setLoggerElementCb, this, _1, _2));
  startLoggerService_ =
      node_->create_service<std_srvs::srv::Trigger>("silo_ros/start_logger", std::bind(&SignalLoggerRos::startLoggerCb, this, _1, _2));
  stopLoggerService_ =
      node_->create_service<std_srvs::srv::Trigger>("silo_ros/stop_logger", std::bind(&SignalLoggerRos::stopLoggerCb, this, _1, _2));
  saveLoggerDataService_ = node_->create_service<signal_logger_msgs::srv::SaveLoggerData>(
      "silo_ros/save_logger_data", std::bind(&SignalLoggerRos::saveLoggerDataCb, this, _1, _2));
  loadLoggerScriptService_ = node_->create_service<signal_logger_msgs::srv::EditLoggerScript>(
      "silo_ros/load_logger_script", std::bind(&SignalLoggerRos::loadLoggerScriptCb, this, _1, _2));
  saveLoggerScriptService_ = node_->create_service<signal_logger_msgs::srv::EditLoggerScript>(
      "silo_ros/save_logger_script", std::bind(&SignalLoggerRos::saveLoggerScriptCb, this, _1, _2));
  isLoggerRunningService_ = node_->create_service<std_srvs::srv::Trigger>("silo_ros/is_logger_running",
                                                                          std::bind(&SignalLoggerRos::isLoggerRunningCb, this, _1, _2));
}

bool SignalLoggerRos::cleanup() {
  signal_logger_std::SignalLoggerStd::cleanup();
  getLoggerConfigurationService_.reset();
  getLoggerElementService_.reset();
  setLoggerElementService_.reset();
  startLoggerService_.reset();
  stopLoggerService_.reset();
  saveLoggerDataService_.reset();
  loadLoggerScriptService_.reset();
  saveLoggerScriptService_.reset();
  isLoggerRunningService_.reset();
  node_.reset();
  return true;
}

signal_logger::TimestampPair SignalLoggerRos::getCurrentTime() {
  // Get time in seconds and nanoseconds
  builtin_interfaces::msg::Time now = rclcpp::convert_rcl_time_to_sec_nanos(node_->now().nanoseconds());
  return {now.sec, now.nanosec};
}

bool SignalLoggerRos::workerSaveData(const std::string& logFileName, const std::string& pathWithPrefix,
                                     const signal_logger::LogFileTypeSet& logfileTypes) {
  if (noCollectDataCallsCopy_ == 0) {
    MELO_WARN_STREAM("[SignalLoggerRos] Could not save logger data! Data count is zero.");
    return false;
  }

  bool success = true;

  // Loop through file types and store data
  for (const auto fileType : logfileTypes) {
    if (fileType == signal_logger::LogFileType::BAG) {
      // Open a new file
      std::string bagFileName = logFileName;
      try {
        bagWriter_.open(bagFileName);
      } catch (std::runtime_error& exception) {
        MELO_WARN_STREAM("[SignalLoggerRos] Could not save bag file! Could not open file " << bagFileName << ".");
        bagWriter_.close();
        success = false;
      }

      // Write bag file
      for (auto& elem : enabledElements_) {
        if (elem->second->getCopyOptions().isSaved()) {
          elem->second->saveDataToLogFile(*timeElement_, noCollectDataCallsCopy_, fileType);
        }
      }

      // Close file
      bagWriter_.close();
    } else {
      success = success && signal_logger_std::SignalLoggerStd::workerSaveData(logFileName, pathWithPrefix, {fileType});
    }
  }

  return success;
}

void SignalLoggerRos::getLoggerConfigurationCb(
    [[maybe_unused]] const signal_logger_msgs::srv::GetLoggerConfiguration::Request::SharedPtr req,
    signal_logger_msgs::srv::GetLoggerConfiguration::Response::SharedPtr res) {
  {
    boost::shared_lock<boost::shared_mutex> getLoggerConfigurationLock(loggerMutex_);
    for (auto& elem : this->logElements_) {
      res->log_element_names.push_back(elem.first);
    }

    res->collect_frequency = options_.updateFrequency_;
    res->logger_namespace = "/log";
    res->script_filepath = options_.collectScriptFileName_;
  }

  if (res->script_filepath.compare(0, 1, "/") != 0) {
    res->script_filepath.insert(0, std::filesystem::current_path().string() + "/");
  }
}

void SignalLoggerRos::getLoggerElementCb(const signal_logger_msgs::srv::GetLoggerElement::Request::SharedPtr req,
                                         signal_logger_msgs::srv::GetLoggerElement::Response::SharedPtr res) {
  signal_logger_msgs::msg::LogElement elem_msg;
  res->success = logElementtoMsg(req->name, elem_msg);
  res->log_element = elem_msg;
}

void SignalLoggerRos::setLoggerElementCb(const signal_logger_msgs::srv::SetLoggerElement::Request::SharedPtr req,
                                         signal_logger_msgs::srv::SetLoggerElement::Response::SharedPtr res) {
  if (isCollectingData_) {
    res->success = false;
  } else {
    res->success = msgToLogElement(req->log_element);
  }
}

void SignalLoggerRos::startLoggerCb([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr req,
                                    std_srvs::srv::Trigger::Response::SharedPtr res) {
  res->success = signal_logger::SignalLoggerBase::startLogger();
}

void SignalLoggerRos::stopLoggerCb([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr req,
                                   std_srvs::srv::Trigger::Response::SharedPtr res) {
  res->success = signal_logger::SignalLoggerBase::stopLogger();
}

void SignalLoggerRos::saveLoggerDataCb(const signal_logger_msgs::srv::SaveLoggerData::Request::SharedPtr req,
                                       signal_logger_msgs::srv::SaveLoggerData::Response::SharedPtr res) {
  signal_logger::LogFileTypeSet types;
  for (const auto filetype : req->logfile_types) {
    switch (filetype) {
      case signal_logger_msgs::srv::SaveLoggerData::Request::LOGFILE_TYPE_BINARY:
        types.insert(signal_logger::LogFileType::BINARY);
        break;
      case signal_logger_msgs::srv::SaveLoggerData::Request::LOGFILE_TYPE_CSV:
        types.insert(signal_logger::LogFileType::CSV);
        break;
      case signal_logger_msgs::srv::SaveLoggerData::Request::LOGFILE_TYPE_BAG:
        types.insert(signal_logger::LogFileType::BAG);
        break;
      default:
        MELO_WARN("Log file type is not known. Do nothing.");
        res->success = false;
        return;
    }
  }

  res->success = signal_logger::SignalLoggerBase::saveLoggerData(types, req->custom_filename);
}

void SignalLoggerRos::isLoggerRunningCb([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr req,
                                        std_srvs::srv::Trigger::Response::SharedPtr res) {
  res->success = this->isCollectingData_;
}

void SignalLoggerRos::loadLoggerScriptCb(const signal_logger_msgs::srv::EditLoggerScript::Request::SharedPtr req,
                                         signal_logger_msgs::srv::EditLoggerScript::Response::SharedPtr res) {
  if (isCollectingData_) {
    res->success = false;
  } else {
    res->success = signal_logger::SignalLoggerBase::readDataCollectScript(req->filepath);
  }
}

void SignalLoggerRos::saveLoggerScriptCb(const signal_logger_msgs::srv::EditLoggerScript::Request::SharedPtr req,
                                         signal_logger_msgs::srv::EditLoggerScript::Response::SharedPtr res) {
  if (isCollectingData_) {
    res->success = false;
  } else {
    res->success = SignalLoggerBase::saveDataCollectScript(req->filepath);
  }
}

bool SignalLoggerRos::logElementtoMsg(const std::string& name, signal_logger_msgs::msg::LogElement& msg) {
  // Shared lock not modifying logger
  boost::shared_lock<boost::shared_mutex> logElementtoMsgLock(loggerMutex_);

  if (logElements_.find(name) == logElements_.end()) {
    return false;
  }

  msg.name = logElements_.at(name)->getOptions().getName();
  msg.is_logged = logElements_.at(name)->isEnabled();
  msg.divider = logElements_.at(name)->getOptions().getDivider();
  msg.buffer_size = logElements_.at(name)->getBuffer().getBufferSize();
  msg.no_items_in_buffer = logElements_.at(name)->getBuffer().noTotalItems();
  msg.no_unread_items_in_buffer = logElements_.at(name)->getBuffer().noUnreadItems();

  switch (logElements_.at(name)->getOptions().getAction()) {
    case signal_logger::LogElementAction::SAVE_AND_PUBLISH:
      msg.action = signal_logger_msgs::msg::LogElement::ACTION_SAVE_AND_PUBLISH;
      break;
    case signal_logger::LogElementAction::SAVE:
      msg.action = signal_logger_msgs::msg::LogElement::ACTION_SAVE;
      break;
    case signal_logger::LogElementAction::PUBLISH:
      msg.action = signal_logger_msgs::msg::LogElement::ACTION_PUBLISH;
      break;
    default:
      MELO_ERROR("Undefined action!");
      break;
  }

  switch (logElements_.at(name)->getBuffer().getBufferType()) {
    case signal_logger::BufferType::FIXED_SIZE:
      msg.buffer_type = signal_logger_msgs::msg::LogElement::BUFFERTYPE_FIXED_SIZE;
      break;
    case signal_logger::BufferType::LOOPING:
      msg.buffer_type = signal_logger_msgs::msg::LogElement::BUFFERTYPE_LOOPING;
      break;
    case signal_logger::BufferType::EXPONENTIALLY_GROWING:
      msg.buffer_type = signal_logger_msgs::msg::LogElement::BUFFERTYPE_EXPONENTIALLY_GROWING;
      break;
    default:
      MELO_ERROR("Undefined buffer type!");
      break;
  }

  return true;
}

bool SignalLoggerRos::msgToLogElement(const signal_logger_msgs::msg::LogElement& msg) {
  boost::unique_lock<boost::shared_mutex> msgToLogElementLock(loggerMutex_);

  auto element_iterator = logElements_.find(msg.name);
  if (element_iterator == logElements_.end()) {
    return false;
  }

  logElements_.at(msg.name)->setIsEnabled(msg.is_logged);

  auto enabled_iterator = std::find(enabledElements_.begin(), enabledElements_.end(), element_iterator);

  if (msg.is_logged) {
    if (enabled_iterator == enabledElements_.end()) {
      enabledElements_.push_back(element_iterator);
    }
  } else {
    if (enabled_iterator != enabledElements_.end()) {
      enabledElements_.erase(enabled_iterator);
    }
  }

  logElements_.at(msg.name)->getOptions().setDivider(msg.divider);
  logElements_.at(msg.name)->getBuffer().setBufferSize(msg.buffer_size);

  switch (msg.action) {
    case signal_logger_msgs::msg::LogElement::ACTION_SAVE_AND_PUBLISH:
      logElements_.at(msg.name)->getOptions().setAction(signal_logger::LogElementAction::SAVE_AND_PUBLISH);
      break;
    case signal_logger_msgs::msg::LogElement::ACTION_SAVE:
      logElements_.at(msg.name)->getOptions().setAction(signal_logger::LogElementAction::SAVE);
      break;
    case signal_logger_msgs::msg::LogElement::ACTION_PUBLISH:
      logElements_.at(msg.name)->getOptions().setAction(signal_logger::LogElementAction::PUBLISH);
      break;
    default:
      MELO_ERROR("Undefined action!");
      break;
  }

  switch (msg.buffer_type) {
    case signal_logger_msgs::msg::LogElement::BUFFERTYPE_FIXED_SIZE:
      logElements_.at(msg.name)->getBuffer().setBufferType(signal_logger::BufferType::FIXED_SIZE);
      break;
    case signal_logger_msgs::msg::LogElement::BUFFERTYPE_LOOPING:
      logElements_.at(msg.name)->getBuffer().setBufferType(signal_logger::BufferType::LOOPING);
      break;
    case signal_logger_msgs::msg::LogElement::BUFFERTYPE_EXPONENTIALLY_GROWING:
      logElements_.at(msg.name)->getBuffer().setBufferType(signal_logger::BufferType::EXPONENTIALLY_GROWING);
      break;
    default:
      MELO_ERROR("Undefined buffer type!");
      break;
  }

  return true;
}

}  // namespace signal_logger_ros
#endif /* ROS2_BUILD */
