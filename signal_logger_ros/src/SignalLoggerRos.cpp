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
  getLoggerElementNamesService_ = nh_.advertiseService("/sl_ros/get_logger_element_names", &SignalLoggerRos::getLoggerElementNames, this);
  getLoggerElementService_ = nh_.advertiseService("/sl_ros/get_logger_element", &SignalLoggerRos::getLoggerElement, this);
  setLoggerElementService_ = nh_.advertiseService("/sl_ros/set_logger_element", &SignalLoggerRos::setLoggerElement, this);
  startLoggerService_ = nh_.advertiseService("/sl_ros/start_logger", &SignalLoggerRos::startLogger, this);
  stopLoggerService_ = nh_.advertiseService("/sl_ros/stop_logger", &SignalLoggerRos::stopLogger, this);
  saveLoggerDataService_ = nh_.advertiseService("/sl_ros/save_logger_data", &SignalLoggerRos::saveLoggerData, this);
  loadLoggerScriptService_ = nh_.advertiseService("/sl_ros/load_logger_script", &SignalLoggerRos::loadLoggerScript, this);
}

SignalLoggerRos::~SignalLoggerRos()
{

}

//! Do nothing for now
bool SignalLoggerRos::workerSaveData(const std::string & logFileName) {
  return true;
}

bool SignalLoggerRos::getLoggerElementNames(signal_logger_msgs::GetLoggerElementNamesRequest& req,
                                            signal_logger_msgs::GetLoggerElementNamesResponse& res) {
  for(auto & elem : this->logElements_)
  {
    res.log_element_names.push_back(elem.first);
  }
  return true;
}

bool SignalLoggerRos::getLoggerElement(signal_logger_msgs::GetLoggerElement::Request& req,
                                       signal_logger_msgs::GetLoggerElement::Response& res)
{
  signal_logger_msgs::LogElement elem_msg;
  res.success = logElementtoMsg(req.name, elem_msg);
  res.log_element = elem_msg;
  return true;
}

bool SignalLoggerRos::setLoggerElement(signal_logger_msgs::SetLoggerElement::Request& req,
                                       signal_logger_msgs::SetLoggerElement::Response& res) {
  res.success = msgToLogElement(req.log_element);
  return true;
}

bool SignalLoggerRos::startLogger(std_srvs::TriggerRequest& req,
                                  std_srvs::TriggerResponse& res) {
  res.success =  SignalLoggerBase::startLogger();
  return true;
}

bool SignalLoggerRos::stopLogger(std_srvs::TriggerRequest& req,
                                 std_srvs::TriggerResponse& res) {
  res.success =  SignalLoggerBase::stopLogger();
  return true;
}

bool SignalLoggerRos::saveLoggerData(std_srvs::TriggerRequest& req,
                                     std_srvs::TriggerResponse& res) {
  res.success = SignalLoggerBase::saveLoggerData();
  return true;
}

bool SignalLoggerRos::loadLoggerScript(signal_logger_msgs::LoadLoggerScriptRequest& req,
                                       signal_logger_msgs::LoadLoggerScriptResponse& res) {
  res.success = SignalLoggerBase::readDataCollectScript(req.filepath);
  return true;
}

bool SignalLoggerRos::logElementtoMsg(const std::string & name, signal_logger_msgs::LogElement & msg)
{
  if( logElements_.find(name) == logElements_.end()) { return false; }

  msg.name = logElements_.at(name)->getName();
  msg.is_logged = logElements_.at(name)->isEnabled();
  msg.divider = logElements_.at(name)->getDivider();
  msg.action = signal_logger_msgs::LogElement::SAVE_AND_PUBLISH_VAR;
  msg.buffer_size = logElements_.at(name)->getBufferSize();
  msg.is_buffer_looping = logElements_.at(name)->isBufferLooping();
  msg.no_items_in_buffer = logElements_.at(name)->noItemsInBuffer();
  msg.no_unread_items_in_buffer = logElements_.at(name)->noUnreadItemsInBuffer();

  return true;
}

bool SignalLoggerRos::msgToLogElement(const signal_logger_msgs::LogElement & msg)
{
  if( logElements_.find(msg.name) == logElements_.end()) { return false; }

  logElements_.at(msg.name)->setIsEnabled(msg.is_logged);
  logElements_.at(msg.name)->setDivider(msg.divider);
  // Publish/save action
  logElements_.at(msg.name)->setBufferSize(msg.buffer_size);
  logElements_.at(msg.name)->setIsBufferLooping(msg.is_buffer_looping);

  return true;
}



} /* namespace signal_logger */
