/*
 * SignalLoggerBase.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/LogElementInterface.hpp"
#include "signal_logger/macro_definitions.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// kindr
#include <kindr/Core>

// eigen
#include <Eigen/Dense>

// stl
#include <typeindex>
#include <mutex>
#include <atomic>

namespace signal_logger {

// Some logger defaults
const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const bool LOGGER_DEFAULT_UPDATE            = false;
const std::string LOGGER_DEFAULT_SCRIPT_FILENAME   = "logger.script";
const std::string LOGGER_PREFIX = "/log";

//! Class that severs as base class for all the loggers and defines the interface for accessing the logger
class SignalLoggerBase {

 public:
  //! Convenience typedefs (They are also used in the macros -> do not delete them)
  typedef Eigen::Matrix< long ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXl;
  typedef Eigen::Matrix< short ,Eigen::Dynamic, Eigen::Dynamic >        MatrixXs;
  typedef Eigen::Matrix< char ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXc;
  typedef Eigen::Matrix< unsigned char ,Eigen::Dynamic, Eigen::Dynamic > MatrixXUc;
  typedef Eigen::Matrix< bool ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXb;
  typedef Eigen::Matrix< std::string ,Eigen::Dynamic, Eigen::Dynamic >  MatrixXstring;
  typedef std::pair<int64_t, int64_t> TimestampPair; // s/ns

  //! Get the logger type at runtime
  enum class LoggerType: unsigned int {
    TypeNone = 0,
        TypeStd,
        TypeRos
  };

 public:
  /** Constructor
   * @param buffer_size size of the internal buffers of the logger elements
   */
  SignalLoggerBase();

  //! Destructor
  virtual ~SignalLoggerBase();

  /** Initializes the logger
   * @param updateFrequency   Update frequency of the controller (frequency of collectLoggerData beeing called)
   * @param samplingFrequency Frequency at which data points should be collected
   * @param samplingTime      Total time that should be recorded (determines the length of the buffer)
   * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
   */
  virtual void initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& collectScriptFileName = std::string{LOGGER_DEFAULT_SCRIPT_FILENAME});

  //! Starts the logger (enable collecting)
  virtual void startLogger();

  //! Stop the logger (disable collecting)
  virtual void stopLogger();

  //! Stop and then restart the logger
  virtual void restartLogger();

  /** Update the logger (added variables are added)
   * @param updateScript, determines if the update script shall be update with the new variables
   */
  virtual void updateLogger(bool updateScript = true);

  //! Do not allow to update the logger
  virtual void lockUpdate();

  //! Collect log data, read data and push it into the buffer
  virtual void collectLoggerData();

  //! Save all the buffered data into a log file
  virtual void saveLoggerData();

  //! Stop the logger and save all the buffered data into a log file
  virtual void stopAndSaveLoggerData();

  //! @return the logger type
  virtual LoggerType getLoggerType() const = 0;

  //! @return the update frequency
  int getUpdateFrequency() const { return updateFrequency_.load(); }

  //! @return the sampling frequency
  virtual int getSamplingFrequency() const { return samplingFrequency_.load(); }

  //! @return the sampling window
  virtual double getSamplingWindow() const { return samplingTime_.load(); }


  template<typename ValueType_>
  void add( const ValueType_ & var,
            const std::string& name,
            const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
            const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
            bool update = LOGGER_DEFAULT_UPDATE)
  {
    MELO_ERROR("Type of signal with name %s is not supported.", name.c_str());
  }

 protected:
  /** Reads collect script and enables all log data
   * @param scriptName filename of the logging script
   */
  bool readDataCollectScript(const std::string & scriptName);

  // Add pure virtual add-functions for every single type
  FOR_ALL_TYPES(ADD_VAR_DEFINITION);
  FOR_EIGEN_TYPES(ADD_EIGEN_VAR_AS_UNDERLYING_TYPE_IMPLEMENTATION);


 protected:
  //! Flag to check if logger is initialized
  std::atomic_bool isInitialized_;
  //! Flag to lock update
  std::atomic_bool isUpdateLocked_;
  //! Flag to collect data
  std::atomic_bool isCollectingData_;
  //! Nr of calls to collect data
  std::atomic_uint noCollectDataCalls_;
  //! Data in buffer
  std::atomic_uint noDataInBuffer_;
  //! Store rate updateFrequency_ / samplingFrequency_
  std::atomic_uint bufferStorageRate_;
  //! Buffer size
  std::size_t bufferSize_;
  //! Collected data script filename
  std::string collectScriptFileName_;
  //! Rate at which collectLoggerData() is called
  std::atomic_uint updateFrequency_;
  //! Rate at which data shall be collected
  std::atomic_uint samplingFrequency_;
  //! Total recording time -> determines length of buffer
  std::atomic<double> samplingTime_;
  //! Map of all log elements
  std::map<std::string, signal_logger::LogElementInterface *> logElements_;

  //! Mutexes
  std::mutex loggerMutex_;
  std::mutex scriptMutex_;
};

FOR_ALL_TYPES(ADD_VAR_TEMPLATE_SPECIFICATIONS);

}

