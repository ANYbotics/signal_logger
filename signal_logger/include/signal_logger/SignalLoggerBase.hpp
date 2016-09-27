/*
 * SignalLoggerBase.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "signal_logger/macro_definitions.hpp"

#include <Eigen/Dense>

#include <kindr/Core>

#include <typeindex>


namespace signal_logger {

const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const bool LOGGER_DEFAULT_UPDATE            = false;
const std::string LOGGER_DEFAULT_FILENAME   = "logger.script";
const std::string LOGGER_PREFIX = "/log";

class SignalLoggerBase {

 public:
  //! Convenience typedefs (They are also used in the macros)
  typedef Eigen::Matrix< long ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXl;
  typedef Eigen::Matrix< short ,Eigen::Dynamic, Eigen::Dynamic >        MatrixXs;
  typedef Eigen::Matrix< char ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXc;
  typedef Eigen::Matrix< unsigned char ,Eigen::Dynamic, Eigen::Dynamic > MatrixXUc;
  typedef Eigen::Matrix< bool ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXb;
  typedef Eigen::Matrix< std::string ,Eigen::Dynamic, Eigen::Dynamic >  MatrixXstring;

  //! First variable is the time in seconds and the second variable is the remaining time in nanoseconds
  typedef std::pair<int64_t, int64_t> TimestampPair;

  enum class LoggerType: unsigned int {
    TypeNone = 0,
        TypeStd,
        TypeRos
  };

 public:
  //! Default constructor
  SignalLoggerBase(std::size_t buffer_size): buffer_size_(buffer_size)  { }

  //! Default destructor
  virtual ~SignalLoggerBase() { }

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
  virtual LoggerType getLoggerType() const = 0;

  //! @return the update frequency
  virtual int getUpdateFrequency() const = 0;

  //! @return the sampling frequency
  virtual int getSamplingFrequency() const = 0;

  //! @return the sampling window
  virtual double getSamplingWindow() const = 0;

  template<typename ValueType_>
  void addVariableToLog(const ValueType_ & var,
                        const std::string& name,
                        const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                        const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                        bool update = LOGGER_DEFAULT_UPDATE)
  {
    printf("Type of signal with name %s is not supported.", name.c_str());
  }

 protected:
  // Add pure virtual add-functions for every single type
  FOR_ALL_TYPES(ADD_VAR_DEFINITION)
  FOR_EIGEN_TYPES(ADD_EIGEN_VAR_AS_UNDERLYING_TYPE_IMPLEMENTATION)


 protected:
 std::size_t buffer_size_;
 std::unordered_map<std::string, signal_logger::LogElementInterface *> log_elements_;

};

FOR_ALL_TYPES(ADD_VAR_TEMPLATE_SPECIFICATIONS)

}

