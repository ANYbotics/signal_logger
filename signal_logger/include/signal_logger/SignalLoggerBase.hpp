/*
 * SignalLoggerBase.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/LogElementTypes.hpp"
#include "signal_logger/LogElementInterface.hpp"
#include "signal_logger/macro_definitions.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// kindr
#include <kindr/Core>

// eigen
#include <Eigen/Dense>

// stl
#include <mutex>
#include <atomic>

namespace signal_logger {

// Some logger element defaults
const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const std::size_t LOGGER_DEFAULT_DIVIDER    = 1;
const LogElementInterface::LogElementAction LOGGER_DEFAULT_ACTION = LogElementInterface::LogElementAction::SAVE_AND_PUBLISH;
const std::size_t LOGGER_DEFAULT_BUFFER_SIZE = 0;
const bool LOGGER_DEFAULT_BUFFER_LOOPING = false;

// Some logger defaults
const std::string LOGGER_DEFAULT_SCRIPT_FILENAME   = "logger.yaml";
const std::string LOGGER_PREFIX = "/log";

//! Class that severs as base class for all the loggers and defines the interface for accessing the logger
class SignalLoggerBase {

 public:

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
   * @param updateFrequency   Update frequency of the controller (frequency of collectLoggerData being called)
   * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
   */
  virtual void initLogger(int updateFrequency, const std::string& collectScriptFileName = std::string{LOGGER_DEFAULT_SCRIPT_FILENAME});

  //! Starts the logger (enable collecting)
  virtual bool startLogger();

  //! Stop the logger (disable collecting)
  virtual bool stopLogger();

  //! Stop and then restart the logger
  virtual bool restartLogger();

  /** Update the logger (added variables are added) */
  virtual bool updateLogger(bool updateScript = false);

  //! Do not allow to update the logger
  virtual void lockUpdate(bool lock = true);

  //! Collect log data, read data and push it into the buffer
  virtual bool collectLoggerData();

  //! Publish a single data point of every element from the buffer
  virtual bool publishData();

  //! Save all the buffered data into a log file
  virtual bool saveLoggerData();

  //! Stop the logger and save all the buffered data into a log file
  virtual bool stopAndSaveLoggerData();

  //! @return the logger type
  virtual LoggerType getLoggerType() const = 0;

  //! @return the update frequency
  int getUpdateFrequency() const { return updateFrequency_.load(); }

  /** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
    * @tparam ValueType_    Data type of the logger element
    * @param var            log variable
    * @param name           name of the log variable
    * @param group          logger group the variable belongs to
    * @param unit           unit of the log variable
    * @param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)
    * @param action         log action of the log variable
    * @param bufferSize     size of the buffer storing log elements
    * @param bufferLooping  determines if the buffer overwrites old values
    */
  template<typename ValueType_>
  void add( const ValueType_ & var,
            const std::string& name,
            const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
            const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
            const std::size_t divider = LOGGER_DEFAULT_DIVIDER,
            const LogElementInterface::LogElementAction & action = LOGGER_DEFAULT_ACTION,
            const std::size_t bufferSize = LOGGER_DEFAULT_BUFFER_SIZE,
            const bool bufferLooping = LOGGER_DEFAULT_BUFFER_LOOPING )
  {
    MELO_ERROR("Type of signal with name %s is not supported.", name.c_str());
  }

 protected:
  /** Reads collect script and enables all log data
   * @param scriptName filename of the logging script
   */
  bool readDataCollectScript(const std::string & scriptName);

  /** Save collect script
   * @param scriptName filename of the logging script
   */
  bool saveDataCollectScript(const std::string & scriptName);

  /** Saves the logger data in a file in a seperate thread
   * @param logFileName filename of the log file
   */
  virtual bool workerSaveData(const std::string & logFileName) = 0;

  //! Add pure virtual add-functions for every single type
  FOR_ALL_TYPES(ADD_VAR_DEFINITION);
  FOR_EIGEN_TYPES(ADD_EIGEN_VAR_AS_UNDERLYING_TYPE_IMPLEMENTATION);

 protected:
  //! Flag to check if logger is initialized
  std::atomic_bool isInitialized_;
  //! Flag to lock update
  std::atomic_bool isUpdateLocked_;
  //! Flag to collect data
  std::atomic_bool isCollectingData_;
  //! Flag to save data
  std::atomic_bool isSavingData_;
  //! Nr of calls to collect data
  std::atomic_uint noCollectDataCalls_;
  //! Collected data script filename
  std::string collectScriptFileName_;
  //! Rate at which collectLoggerData() is called
  std::atomic_uint updateFrequency_;
  //! Map of all log elements
  std::map<std::string, signal_logger::LogElementInterface *> logElements_;

  //! Mutexes
//  std::mutex loggerMutex_;
  std::mutex scriptMutex_;
};

//! Add template specifications
FOR_ALL_TYPES(ADD_VAR_TEMPLATE_SPECIFICATIONS);

}

