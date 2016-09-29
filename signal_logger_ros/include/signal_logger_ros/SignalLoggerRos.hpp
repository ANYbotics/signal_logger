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

 protected:
  FOR_ALL_TYPES(ADD_ROS_VAR_IMPLEMENTATION);

 private:
  ros::NodeHandle nh_;
};

} /* namespace signal_logger */
