/**
* @file 	  signal_logger.cpp
* @author 	Christian Gehring, C. Dario Bellicoso, Gabriel Hottiger
* @date		  June 26, 2013
*/

// signal_logger
#include "signal_logger/signal_logger.hpp"
#include "signal_logger/SignalLoggerNone.hpp"

// STL
#include <memory>

namespace signal_logger {

//! Initialize logger with standard logger.
std::shared_ptr<SignalLoggerBase> logger(new SignalLoggerNone());

void setSignalLoggerNone() {
  logger.reset(new signal_logger::SignalLoggerNone());
}

void setSignalLoggerStd() {
  logger.reset(new signal_logger_std::SignalLoggerStd());
}

#ifdef SILO_USE_ROS
void setSignalLoggerRos(ros::NodeHandle* nh, bool saveToBag) {
  logger.reset(new signal_logger_ros::SignalLoggerRos(nh));
}
#endif



} /* namespace signal_logger */
