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
#include <condition_variable>
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
   *  @param logFileTypes types of the log file
   */
  virtual bool saveLoggerData(const LogFileTypeSet & logfileTypes);

  /** Stop the logger and save all the buffered data into log files
   *  @param logFileTypes types of the log file
   */
  virtual bool stopAndSaveLoggerData(const LogFileTypeSet & logfileTypes);

  //! Cleanup logger
  virtual bool cleanup();

  //! Returns if an element is logged with this name
  virtual bool hasElement(const std::string & name);

  virtual bool enableElement(const std::string & name);
  virtual bool disableElement(const std::string & name);

  virtual bool enableNamespace(const std::string & ns);
  virtual bool disableNamespace(const std::string & ns);

  bool setElementBufferSize(const std::string & name, const std::size_t size);
  bool setElementBufferType(const std::string & name, const BufferType type);
  bool setElementDivider(const std::string & name, const std::size_t divider);
  bool setElementAction(const std::string & name, const LogElementAction action);

  //! Get log element (throws std::out_of_range() if no element with name 'name' was added to the logger)
  virtual const LogElementInterface & getElement(const std::string & name);

  //! Get log element (throws std::out_of_range() if no element with name 'name' was added to the logger, or it it of different type)
  template<typename ValueType_>
  vector_type<ValueType_> readNewValues(const std::string & name) {
    boost::shared_lock<boost::shared_mutex> lockLogger(loggerMutex_);

    if(!hasElement(name) || logElements_[name]->getType() !=  typeid(ValueType_) ) {
      throw std::out_of_range("[SignalLoggerBase]::readNewValues(): Element " + name +
          " was not added to the logger or is not of type " + std::type_index(typeid(ValueType_)).name() + "!");
    }

    return static_cast< LogElementBase<ValueType_>* >(logElements_[name].get())->readNewValues();
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
   * @param logfileTypes types of the log files
   */
  virtual bool workerSaveData(const std::string & logFileName, const LogFileTypeSet & logfileTypes) = 0;

  /** Initializes the pointer to the log element */
  virtual void initTimeLogElement();

  /** Returns the current time
   * @return current time
   */
  virtual signal_logger::TimestampPair getCurrentTime();

 private:
  /** Wraps function workerSaveData to do common preparations and shutdown
   * @param logFileTypse types of the log files
   */
  bool workerSaveDataWrapper(const LogFileTypeSet & logfileTypes);

  /** Wait until logger can be started and start logger
   */
  bool workerStartLogger();

  //! Comparison operator, get element with largest scaled buffer size
  struct maxScaledBufferSize {
    /*** Defines the comparison operator
     *   @param i  first element to compare
     *   @param j  second element to compare
     *   @return fun(i) < fun(j)
     */
    bool operator() (const LogElementMapIterator& i, const LogElementMapIterator& j)
    {
      return (i->second->getOptions().getDivider()*i->second->getBuffer().getBufferSize()) <
             (j->second->getOptions().getDivider()*j->second->getBuffer().getBufferSize());
    }
  };

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
  //! Flag if data should be published
  std::atomic_bool shouldPublish_;

  //-- Mutexes
  //! Init, add, update, start, stop and cleanup should be called in sequence
  boost::shared_mutex loggerMutex_;
  //! Allow call to save logger data only once
  std::mutex saveLoggerDataMutex_;

};

} // end namespace signal_logger
