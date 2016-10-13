/*
 * SignalLoggerNone.hpp
 *
 *  Created on: Oct 10, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/SignalLoggerBase.hpp"
#include "signal_logger/macro_definitions.hpp"

namespace signal_logger {

class SignalLoggerNone : public SignalLoggerBase
{
 public:
  //! Constructor
  SignalLoggerNone() { }

  //! Destructor
  virtual ~SignalLoggerNone() { }

  /** Initializes the logger
   * @param updateFrequency   Update frequency of the controller (frequency of collectLoggerData being called)
   * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
   */
  virtual void initLogger(int updateFrequency, const std::string& collectScriptFileName) { }

  //! Starts the logger (enable collecting)
  virtual bool startLogger() { return true; }

  //! Stop the logger (disable collecting)
  virtual bool stopLogger() { return true; }

  //! Stop and then restart the logger
  virtual bool restartLogger() { return true; }

  /** Update the logger (added variables are added) */
  virtual bool updateLogger(bool updateScript = false) { return true; }

  //! Lock or unlock update logger
  virtual void lockUpdate(bool lock = true) { }

  //! Collect log data, read data and push it into the buffer
  virtual bool collectLoggerData() { return true; }

  //! Publish a single data point of every element from the buffer
  virtual bool publishData() { return true; }

  //! Save all the buffered data into a log file
  virtual bool saveLoggerData() { return true; }

  //! Stop the logger and save all the buffered data into a log file
  virtual bool stopAndSaveLoggerData() { return true; }

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeNone; }

 protected:
  /** Saves the logger data in a file in a seperate thread
   * @param logFileName filename of the log file
   */
  virtual bool workerSaveData(const std::string & logFileName) { return true; };

  //! Add empty implementation of add-functions for every single type
  FOR_ALL_TYPES(ADD_NONE_VAR_IMPLEMENTATION);

};

}

