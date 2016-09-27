/*
 * Logger.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <signal_logger_std/LogElementStd.hpp>

#include "signal_logger_std/macro_definitions.hpp"
#include "signal_logger/macro_definitions.hpp"

#include "signal_logger/SignalLoggerBase.hpp"

#include <unordered_map>

#include <ros/node_handle.h>

namespace signal_logger_std {

class Logger : public signal_logger::SignalLoggerBase
{
 public:
  Logger(std::size_t buffer_size): signal_logger::SignalLoggerBase(buffer_size), file_() {}
  virtual ~Logger() {}

  void collectData()
  {
    for(auto & elem : log_elements_) elem.second->collectData();
  }

  void publishData()
  {
    std::string filename = "mydata3.txt";
    file_.open(filename, std::ios::out | std::ios::app);
    file_ << "Begin file";
    for(auto & elem : log_elements_) {
      elem.second->writeHeaderToLogFile();
    }
    file_.close();
    file_.open(filename, std::ios::out | std::ios::app | std::ios::binary);
    for(auto & elem : log_elements_) {
      std::cout << " Publish "<<elem.second->getName()<<std::endl;
      elem.second->writeDataToLogFile();
    }

    file_.close();
  }

  /** Initializes the logger
     * @param updateFrequency   Update frequency of the controller (frequency of collectLoggerData beeing called)
     * @param samplingFrequency Frequency at which data points should be collected
     * @param samplingTime      Total time that should be recorded (determines the length of the buffer)
     * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
     */
    virtual void initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName = std::string{LOGGER_DEFAULT_FILENAME}) { }

    //! Starts the logger (enable collecting)
    virtual void startLogger() { }

    //! Stop the logger (disable collecting)
    virtual void stopLogger() { }

    /** Update the logger (added variables are added)
     * @param updateScript, determines if the update script shall be update with the new variables
     */
    virtual void updateLogger(bool updateScript = true) { }

    //! Do not allow to update the logger
    virtual void lockUpdate() { }

    //! Collect log data, read data and push it into the buffer
    virtual void collectLoggerData() { }

    //! Save all the buffered data into a log file
    virtual void saveLoggerData() { }

    //! Stop the logger and save all the buffered data into a log file
    virtual void stopAndSaveLoggerData() { }

    //! Stop and then restart the logger
    virtual void restartLogger() { }

    //! @return the logger type
    virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeNone; }

    //! @return the update frequency
    virtual int getUpdateFrequency() const { return 0; }

    //! @return the sampling frequency
    virtual int getSamplingFrequency() const { return 0; }

    //! @return the sampling window
    virtual double getSamplingWindow() const { return 0.0; }


protected:
  FOR_ALL_TYPES(ADD_VAR_IMPLEMENTATION)

 private:
  std::ofstream file_;
};

} /* namespace signal_logger */
