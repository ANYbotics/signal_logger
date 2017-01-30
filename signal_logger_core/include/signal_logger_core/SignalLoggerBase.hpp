/*!
 * @file     SignalLoggerBase.hpp
 * @author   Gabriel Hottiger, Christian Gehring
 * @date     Sep 26, 2016
 * @brief    Base class for signal loggers.
 */

#pragma once

// signal logger
#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/LogElementBase.hpp"
#include "signal_logger_core/LogElementInterface.hpp"
#include "signal_logger_core/SignalLoggerOptions.hpp"
#include "signal_logger_core/typedefs.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// eigen
#include <Eigen/Dense>

// boost
#include <boost/thread.hpp>

// stl
#include <mutex>
#include <atomic>
#include <memory>
#include <unordered_map>

namespace signal_logger {

//! Class that severs as base class for all the loggers
class SignalLoggerBase {
 protected:
  //! Log element map types
  using LogPair = std::pair<std::string, std::unique_ptr<LogElementInterface>>;
  using LogElementMap = std::unordered_map<std::string, std::unique_ptr<LogElementInterface>>;
  using LogElementMapIterator = LogElementMap::iterator;

 public:
  /** Constructor
   * @param loggerPrefix prefix to the logger variables
   */
  SignalLoggerBase();

  //! Destructor
  virtual ~SignalLoggerBase();

  /** Initializes the logger
   * @param options Signal logger options
   */
  virtual void initLogger(const SignalLoggerOptions& options);

  //! Starts the logger (enable collecting)
  virtual bool startLogger();

  //! Stop the logger (disable collecting)
  virtual bool stopLogger();

  //! Stop and then restart the logger
  virtual bool restartLogger();

  /** Update the logger (added variables are added) */
  virtual bool updateLogger();

  /** Save logger script **/
  virtual bool saveLoggerScript();

  //! Collect log data, read data and push it into the buffer
  virtual bool collectLoggerData();

  //! Publish a single data point of every element from the buffer
  virtual bool publishData();

  /** Save all the buffered data into a log file
   *  @param logFileType type of the log file
   */
  virtual bool saveLoggerData(LogFileType logfileType = LogFileType::BINARY);

  //! Stop the logger and save all the buffered data into a log file
  virtual bool stopAndSaveLoggerData();

  //! Cleanup logger
  virtual bool cleanup();

  //! @return the update frequency
  unsigned int getUpdateFrequency() const;

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
   * @param logFileType type of the log file
   */
  virtual bool workerSaveData(const std::string & logFileName, LogFileType logfileType) = 0;

  /** Resets the pointer to the logelement
   * @param buffertype type of the time buffer
   * @param maxLogTime maximal time logging
   */
  virtual bool resetTimeLogElement(signal_logger::BufferType buffertype, double maxLogTime);

  /** Returns the current time
   * @return current time
   */
  virtual signal_logger::TimestampPair getCurrentTime();

 private:
  /** Wraps function workerSaveData to do common preparations and shutdown
   * @param logFileType type of the log file
   */
  bool workerSaveDataWrapper(LogFileType logfileType);

  /** Wait until logger can be started and start logger
   */
  bool workerStartLogger();


 protected:
  //! Logger Options
  SignalLoggerOptions options_;
  //! Nr of calls to collect data
  std::atomic_uint noCollectDataCalls_;
  //! Copy of nr of calls to collect data for saving
  std::atomic_uint noCollectDataCallsCopy_;

  //-- Log Elements

  //! Map of log elements (excluding time, elements added after last updateLogger() call)
  LogElementMap logElements_;
  //! Vector of iterators in logElements_ map
  std::vector<LogElementMapIterator> enabledElements_;
  //! Map of all log elements added since last call to updateLogger()
  LogElementMap logElementsToAdd_;

  //! Time variable
  TimestampPair logTime_;
  //! Corresponding time log element
  std::shared_ptr<LogElementBase<TimestampPair>> timeElement_;

  //-- Execution Flags

  //! Flag to check if logger is initialized
  std::atomic_bool isInitialized_;
  //! Flag to collect data
  std::atomic_bool isCollectingData_;
  //! Flag to save data
  std::atomic_bool isSavingData_;
  //! Flag to save data
  std::atomic_bool isCopyingBuffer_;
  //! Flag is starting in different thread
  std::atomic_bool isStarting_;

  //-- Mutexes
  std::mutex startLoggerMutex_;
  std::mutex updateLoggerMutex_;
  std::mutex collectDataMutex_;
  std::mutex saveDataMutex_;
  std::mutex publishDataMutex_;

  boost::shared_mutex elementsMapMutex_;
  boost::shared_mutex newElementsMapMutex_;
  boost::shared_mutex timeElementMutex_;

 private:
  //! Comparison operator, get element with largest scaled buffer size
  struct maxScaledBufferSize {
    /*** Defines the comparison operator
     *   @param i  first element to compare
     *   @param j  second element to compare
     *   @return fun(i) < fun(j)
     */
    bool operator() (const LogElementMapIterator& i, const LogElementMapIterator& j)
    {
      return (i->second->getDivider()*i->second->getBufferSize()) <
             (j->second->getDivider()*j->second->getBufferSize());
    }
  };

};

} // end namespace signal_logger
