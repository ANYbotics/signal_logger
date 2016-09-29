/*
 * SignalLoggerRos.cpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#include "signal_logger_ros/SignalLoggerRos.hpp"

namespace signal_logger_ros {

SignalLoggerRos::SignalLoggerRos():
        signal_logger::SignalLoggerBase(),
        nh_()
{

}

SignalLoggerRos::~SignalLoggerRos()
{

}

//! Do nothing for now
bool SignalLoggerRos::workerSaveData(const std::string & logFileName) {
  return true;
}

} /* namespace signal_logger */
