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
  loggerInfoService_ = nh_.advertiseService("/get_log_element_list", &SignalLoggerRos::getLoggerInfo, this);
  getLogElementService_ = nh_.advertiseService("/get_log_element", &SignalLoggerRos::getLogElement, this);
  setLogElementService_ = nh_.advertiseService("/set_log_element", &SignalLoggerRos::setLogElement, this);
}

SignalLoggerRos::~SignalLoggerRos()
{

}

//! Do nothing for now
bool SignalLoggerRos::workerSaveData(const std::string & logFileName) {
  return true;
}

bool SignalLoggerRos::getLoggerInfo(signal_logger_msgs::GetLoggerInfo::Request& req,
                                    signal_logger_msgs::GetLoggerInfo::Response& res) {

  signal_logger_msgs::LogElement elem_msg;
  for(auto & elem : this->logElements_)
  {
    if(elem.second->isEnabled()) {
      res.active_log_elements.push_back(elem.first);
    }
    else {
      res.inactive_log_elements.push_back(elem.first);
    }
  }
  return true;
}

bool SignalLoggerRos::getLogElement(signal_logger_msgs::GetLoggerElement::Request& req,
                                    signal_logger_msgs::GetLoggerElement::Response& res)
{
  signal_logger_msgs::LogElement elem_msg;
  res.success = logElementtoMsg(req.name, elem_msg);
  res.log_element = elem_msg;
  return true;
}

bool SignalLoggerRos::setLogElement(signal_logger_msgs::SetLoggerElement::Request& req,
                                    signal_logger_msgs::SetLoggerElement::Response& res) {
  res.success = msgToLogElement(req.log_element);
  return true;
}

bool SignalLoggerRos::logElementtoMsg(const std::string & name, signal_logger_msgs::LogElement & msg)
{
  if( logElements_.find(name) == logElements_.end()) { return false; }

  msg.name = logElements_.at(name)->getName();
  msg.buffer_size = logElements_.at(name)->getBufferSize();
  msg.divider = logElements_.at(name)->getDivider();
  msg.is_buffer_full = logElements_.at(name)->isBufferFull();
  msg.is_buffer_looping = true;
  msg.action = signal_logger_msgs::LogElement::SAVE_AND_PUBLISH_VAR;

  return true;
}

bool SignalLoggerRos::msgToLogElement(const signal_logger_msgs::LogElement & msg)
{
  if( logElements_.find(msg.name) == logElements_.end()) { return false; }

  logElements_.at(msg.name)->setBufferSize(msg.buffer_size);
  logElements_.at(msg.name)->setDivider(msg.divider);

  return true;
}



} /* namespace signal_logger */
