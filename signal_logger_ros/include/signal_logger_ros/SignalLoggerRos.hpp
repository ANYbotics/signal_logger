/*
 * SignalLoggerRos.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger_ros/macro_definitions.hpp"
#include "signal_logger_std/SignalLoggerStd.hpp"

// msgs
#include "signal_logger_msgs/GetLoggerElementNames.h"
#include "signal_logger_msgs/GetLoggerElement.h"
#include "signal_logger_msgs/SetLoggerElement.h"
#include "signal_logger_msgs/LoadLoggerScript.h"
#include <std_srvs/Trigger.h>

namespace signal_logger_ros {

class SignalLoggerRos : public signal_logger_std::SignalLoggerStd
{
 public:
  SignalLoggerRos();
  virtual ~SignalLoggerRos();

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeRos; }

  //! Services needed by the gui
  bool getLoggerElementNames(signal_logger_msgs::GetLoggerElementNamesRequest& req,
                             signal_logger_msgs::GetLoggerElementNamesResponse& res);

  bool getLoggerElement(signal_logger_msgs::GetLoggerElementRequest& req,
                        signal_logger_msgs::GetLoggerElementResponse& res);

  bool setLoggerElement(signal_logger_msgs::SetLoggerElementRequest& req,
                        signal_logger_msgs::SetLoggerElementResponse& res);

  bool startLogger(std_srvs::TriggerRequest& req,
                   std_srvs::TriggerResponse& res);

  bool stopLogger(std_srvs::TriggerRequest& req,
                  std_srvs::TriggerResponse& res);

  bool saveLoggerData(std_srvs::TriggerRequest& req,
                      std_srvs::TriggerResponse& res);

  bool isLoggerRunning(std_srvs::TriggerRequest& req,
                       std_srvs::TriggerResponse& res);

  bool loadLoggerScript(signal_logger_msgs::LoadLoggerScriptRequest& req,
                        signal_logger_msgs::LoadLoggerScriptResponse& res);

  bool logElementtoMsg(const std::string & name, signal_logger_msgs::LogElement & msg);

  bool msgToLogElement(const signal_logger_msgs::LogElement & msg);

 protected:
  FOR_ALL_TYPES(ADD_ROS_VAR_IMPLEMENTATION);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer getLoggerElementNamesService_;
  ros::ServiceServer getLoggerElementService_;
  ros::ServiceServer setLoggerElementService_;
  ros::ServiceServer startLoggerService_;
  ros::ServiceServer stopLoggerService_;
  ros::ServiceServer saveLoggerDataService_;
  ros::ServiceServer loadLoggerScriptService_;
  ros::ServiceServer isLoggerRunningService_;

};

} /* namespace signal_logger */
