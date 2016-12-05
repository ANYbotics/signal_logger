/*
 * SignalLoggerRos.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger_std/SignalLoggerStd.hpp"
#include <signal_logger_ros/LogElementRos.hpp>

// msgs
#include "signal_logger_msgs/GetLoggerConfiguration.h"
#include "signal_logger_msgs/GetLoggerElement.h"
#include "signal_logger_msgs/SetLoggerElement.h"
#include "signal_logger_msgs/LoadLoggerScript.h"
#include <std_srvs/Trigger.h>

namespace signal_logger_ros {

class SignalLoggerRos : public signal_logger_std::SignalLoggerStd
{
 public:
  SignalLoggerRos(ros::NodeHandle * nh, bool saveToBagFile = false);
  virtual ~SignalLoggerRos();

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
  template<typename ValueType_>
  void add( const ValueType_ * const var,
            const std::string & name,
            const std::string & group       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit        = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
            const signal_logger::LogElementAction action   = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize    = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const signal_logger::BufferType bufferType     = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
  {
    std::string elementName = std::string{signal_logger::SignalLoggerBase::LOGGER_DEFAULT_PREFIX} + "/" + group + "/" + name;
    logElements_[elementName].reset(new LogElementRos<ValueType_>(var, elementName , unit, divider, action, bufferSize,
                                                                  bufferType, &headerStream_, &dataStream_, nh_, bagWriter_));
  }

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeRos; }

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName);

  //! Services needed by the gui
  bool getLoggerConfiguration(signal_logger_msgs::GetLoggerConfigurationRequest& req,
                             signal_logger_msgs::GetLoggerConfigurationResponse& res);

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

  virtual bool cleanup();

 private:
  ros::NodeHandle* nh_;
  bool saveToBagFile_;
  std::shared_ptr<bageditor::BagWriter> bagWriter_;
  ros::ServiceServer getLoggerConfigurationService_;
  ros::ServiceServer getLoggerElementService_;
  ros::ServiceServer setLoggerElementService_;
  ros::ServiceServer startLoggerService_;
  ros::ServiceServer stopLoggerService_;
  ros::ServiceServer saveLoggerDataService_;
  ros::ServiceServer loadLoggerScriptService_;
  ros::ServiceServer isLoggerRunningService_;

};

} /* namespace signal_logger */
