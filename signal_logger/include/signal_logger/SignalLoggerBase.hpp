/*
 * SignalLoggerBase.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/LogElementTypes.hpp"
#include "signal_logger/LogElementBase.hpp"
#include "signal_logger/LogElementInterface.hpp"
#include "signal_logger/macro_definitions.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// eigen
#include <Eigen/Dense>

// stl
#include <mutex>
#include <atomic>
#include <memory>
#include <unordered_map>

namespace signal_logger {

// Forward log element action to signal logger namespace
using LogPair = std::pair<std::string, std::shared_ptr<signal_logger::LogElementInterface>>;



//! Class that severs as base class for all the loggers
class SignalLoggerBase {
 public:
  // Some logger element defaults
  static constexpr const char* LOG_ELEMENT_DEFAULT_GROUP_NAME   = "/log/";
  static constexpr const char* LOG_ELEMENT_DEFAULT_UNIT         = "-";
  static constexpr std::size_t LOG_ELEMENT_DEFAULT_DIVIDER      = 1;
  static constexpr LogElementAction LOG_ELEMENT_DEFAULT_ACTION  = LogElementAction::NONE;
  static constexpr std::size_t LOG_ELEMENT_DEFAULT_BUFFER_SIZE  = 0;
  static constexpr BufferType LOG_ELEMENT_DEFAULT_BUFFER_TYPE   = BufferType::FIXED_SIZE;

  // Some logger defaults
  static constexpr const char* LOGGER_DEFAULT_SCRIPT_FILENAME   = "logger.yaml";
  static constexpr const char* LOGGER_DEFAULT_PREFIX            = "/log";

  using LogElementMap = std::unordered_map<std::string, std::shared_ptr<signal_logger::LogElementInterface>>;
  using LogElementMapIterator = std::unordered_map<std::string, std::shared_ptr<signal_logger::LogElementInterface>>::iterator;

 public:
  //! Get the logger type at runtime
  enum class LoggerType: unsigned int {
    TypeNone = 0,
    TypeStd  = 1,
    TypeRos  =2
  };

 public:
  /** Constructor
   * @param loggerPrefix prefix to the logger variables
   */
  SignalLoggerBase(const std::string & loggerPrefix = LOGGER_DEFAULT_PREFIX);

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

  //! Lock or unlock update logger
  virtual void lockUpdate(bool lock = true);

  //! Collect log data, read data and push it into the buffer
  virtual bool collectLoggerData();

  //! Publish a single data point of every element from the buffer
  virtual bool publishData();

  //! Save all the buffered data into a log file
  virtual bool saveLoggerData();

  //! Stop the logger and save all the buffered data into a log file
  virtual bool stopAndSaveLoggerData();

  //! Cleanup logger
  virtual bool cleanup();

  //! @return the logger type
  virtual LoggerType getLoggerType() const = 0;

  //! @return the update frequency
  int getUpdateFrequency() const { return updateFrequency_.load(); }

  /** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
    * @tparam ValueType_       Data type of the logger element
    * @param  var              log variable
    * @param  name             name of the log variable
    * @param  group            logger group the variable belongs to
    * @param  unit             unit of the log variable
    * @param  divider          divider is defining the update frequency of the logger element (ctrl_freq/divider)
    * @param  action           log action of the log variable
    * @param  bufferSize       size of the buffer storing log elements
    * @param  isBufferLooping  determines if the buffer overwrites old values
    */
  template<typename ValueType_>
  void add( const ValueType_ & var,
            const std::string & name,
            const std::string & group       = LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit        = LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider       = LOG_ELEMENT_DEFAULT_DIVIDER,
            const LogElementAction action   = LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize    = LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const BufferType bufferType     = LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
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
  //! Logger prefix
  std::string loggerPrefix_;
  //! List of enable iterators
  std::unordered_map<std::string, LogElementMapIterator> enabledElements_;
  //! Map of all log elements
  LogElementMap logElements_;
  //! Time variable
  TimestampPair logTime_;
  //! Corresponding time log element
  std::shared_ptr<signal_logger::LogElementBase<signal_logger::TimestampPair>> timeElement_;
  //! Mutexes
  std::mutex scriptMutex_;
};

//! Add template specifications
FOR_ALL_TYPES(ADD_VAR_TEMPLATE_SPECIFICATIONS);

}

