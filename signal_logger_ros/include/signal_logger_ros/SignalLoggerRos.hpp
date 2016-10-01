/*
 * SignalLoggerRos.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger_ros/macro_definitions.hpp"
#include "signal_logger/SignalLoggerBase.hpp"

// msgs
#include "signal_logger_msgs/GetLoggerInfo.h"
#include "signal_logger_msgs/GetLoggerElement.h"
#include "signal_logger_msgs/SetLoggerElement.h"

namespace signal_logger_ros {

class SignalLoggerRos : public signal_logger::SignalLoggerBase
{
 public:
  SignalLoggerRos();
  virtual ~SignalLoggerRos();

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName);

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeRos; }

  //! Services needed by the gui
  bool getLoggerInfo(signal_logger_msgs::GetLoggerInfo::Request& req,
                     signal_logger_msgs::GetLoggerInfo::Response& res);

  bool getLogElement(signal_logger_msgs::GetLoggerElement::Request& req,
                     signal_logger_msgs::GetLoggerElement::Response& res);

  bool setLogElement(signal_logger_msgs::SetLoggerElement::Request& req,
                     signal_logger_msgs::SetLoggerElement::Response& res);

  bool logElementtoMsg(const std::string & name, signal_logger_msgs::LogElement & msg);

  bool msgToLogElement(const signal_logger_msgs::LogElement & msg);

 protected:
  FOR_ALL_TYPES(ADD_ROS_VAR_IMPLEMENTATION);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer loggerInfoService_;
  ros::ServiceServer getLogElementService_;
  ros::ServiceServer setLogElementService_;

};

} /* namespace signal_logger */
