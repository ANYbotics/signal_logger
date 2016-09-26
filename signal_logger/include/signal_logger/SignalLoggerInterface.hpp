/*
 * SignalLoggerInterface.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <string>
#include "signal_logger/macro_definitions.hpp"

const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const bool LOGGER_DEFAULT_UPDATE            = false;
const std::string LOGGER_DEFAULT_FILENAME   = "logger.script";

namespace signal_logger {

class SignalLoggerInterface {

 public:
  enum class LoggerType: unsigned int {
    TypeNone = 0,
        TypeStd,
        TypeRos
  };

 public:
  //! Constructor
  SignalLoggerInterface()
 {

 }

  //! Destructor
  virtual ~SignalLoggerInterface()
  {

  }

  //! Interface

  /** Initializes the logger
   * @param updateFrequency   Update frequency of the controller (frequency of collectLoggerData beeing called)
   * @param samplingFrequency Frequency at which data points should be collected
   * @param samplingTime      Total time that should be recorded (determines the length of the buffer)
   * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
   */
  virtual void initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName = std::string{LOGGER_DEFAULT_FILENAME}) = 0;

  //! Starts the logger (enable collecting)
  virtual void startLogger() = 0;

  //! Stop the logger (disable collecting)
  virtual void stopLogger() = 0;

  /** Update the logger (added variables are added)
   * @param updateScript, determines if the update script shall be update with the new variables
   */
  virtual void updateLogger(bool updateScript = true) = 0;

  //! Do not allow to update the logger
  virtual void lockUpdate() = 0;

  //! Collect log data, read data and push it into the buffer
  virtual void collectLoggerData() = 0;

  //! Save all the buffered data into a log file
  virtual void saveLoggerData() = 0;

  //! Stop the logger and save all the buffered data into a log file
  virtual void stopAndSaveLoggerData() = 0;

  //! Stop and then restart the logger
  virtual void restartLogger() = 0;

  //! @return the logger type
  virtual SignalLoggerInterface::LoggerType getLoggerType() const = 0;

  //! @return the update frequency
  virtual int getUpdateFrequency() const = 0;

  //! @return the sampling frequency
  virtual int getSamplingFrequency() const = 0;

  //! @return the sampling window
  virtual double getSamplingWindow() const = 0;


  FOR_PRIMITIVE_TYPES(DEFINITION_ADD_VAR)
  FOR_EIGEN_TYPES(DEFINITION_ADD_VAR)
  FOR_EIGEN_TYPES(FORWARD_ADD_VAR_EIGEN_NAMES)
  FOR_KINDR_TYPES(DEFINITION_ADD_VAR)

};

}

