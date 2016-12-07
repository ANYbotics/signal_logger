/*
 * SignalLoggerBase.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/LogElementBase.hpp"
#include "signal_logger_core/LogElementInterface.hpp"

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

//! Class that severs as base class for all the loggers
class SignalLoggerBase {
 public:
  //! Some logger element defaults
  static constexpr const char* LOG_ELEMENT_DEFAULT_GROUP_NAME   = "/log/";
  static constexpr const char* LOG_ELEMENT_DEFAULT_UNIT         = "-";
  static constexpr std::size_t LOG_ELEMENT_DEFAULT_DIVIDER      = 1;
  static constexpr LogElementAction LOG_ELEMENT_DEFAULT_ACTION  = LogElementAction::NONE;
  static constexpr std::size_t LOG_ELEMENT_DEFAULT_BUFFER_SIZE  = 1000;
  static constexpr BufferType LOG_ELEMENT_DEFAULT_BUFFER_TYPE   = BufferType::LOOPING;

  //! Some logger defaults
  static constexpr const double LOGGER_DEFAULT_MAXIMUM_LOG_TIME   = 0.0;
  static constexpr const double LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME   = 10.0;
  static constexpr const char* LOGGER_DEFAULT_SCRIPT_FILENAME   = "logger.yaml";
  static constexpr const char* LOGGER_DEFAULT_PREFIX            = "/log";

  //! Log element map types
  using LogPair = std::pair<std::string, std::shared_ptr<signal_logger::LogElementInterface>>;
  using LogElementMap = std::unordered_map<std::string, std::shared_ptr<signal_logger::LogElementInterface>>;
  using LogElementMapIterator = LogElementMap::iterator;

 public:
  //! Get the logger type at runtime
  enum class LoggerType: unsigned int {
    TypeNone = 0,/*!< 0 */
    TypeStd  = 1,/*!< 1 */
    TypeRos  = 2/*!< 2 */
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
   * @param maxLogTime        If a maximal log time is set, then the time buffer has fixed size
   * @param logScriptFileName Filename of the log script, this specifies which topics shall be logged
   */
  virtual void initLogger(int updateFrequency, const double maxLogTime = LOGGER_DEFAULT_MAXIMUM_LOG_TIME, const std::string& collectScriptFileName = std::string{LOGGER_DEFAULT_SCRIPT_FILENAME});

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

  /** Resets the pointer to the logelement
   * @param buffertype type of the time buffer
   * @param maxLogTime maximal time logging
   */
  virtual bool resetTimeLogElement(signal_logger::BufferType buffertype, double maxLogTime = LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME);

  /** Returns the current time
   * @return current time
   */
  virtual signal_logger::TimestampPair getCurrentTime();

 private:
  bool workerSaveDataWrapper(const std::string & logFileName);


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
  //! Logging length
  double maxLoggingTime_;
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
  std::mutex collectMutex_;


 private:
  //! Comparison operator, get element with largest scaled buffer size
  struct maxScaledBufferSize {
    /*** Defines the comaprison operator
     *   @param i  first element to compare
     *   @param j  second element to compare
     *   @return fun(i) < fun(j)
     */
    bool operator() (const std::pair<std::string, LogElementMapIterator> & i, const std::pair<std::string, LogElementMapIterator> & j)
    {
      return (i.second->second->getDivider()*i.second->second->getBufferSize()) <
             (j.second->second->getDivider()*j.second->second->getBufferSize());
    }
  };

};

} // end namespace signal_logger