/*!
 * @file     LogElementInterface.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    A class that defines the basic interface of logger elements.
 */


#pragma once

// signal logger
#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/Buffer.hpp"
#include "signal_logger_core/typedefs.hpp"

// STL
#include <string>
#include <mutex>

namespace signal_logger {

//! Forward declaration of LogElementBase
template <typename ValueType_>
class LogElementBase;

/**
 *  A list of pointers to this class can be stored in the logger, since the class itself is not templated.
 *  \brief Basic interface of logger elements
 */
//! A class that defines the basic interface of logger elements.
class LogElementInterface
{
 public:
  //! Default constructor
  LogElementInterface() { }

  //! Destructor
  virtual ~LogElementInterface() { }

  //! Reads pointer and pushes the data into the buffer
  virtual void collectData() = 0;

  /** Reads buffer and processes data (probably called from different thread)
   *  @param time                 time log element
   *  @param noCollectDataCalls   number of collectLoggerData calls, this allows time synchronization for publishing
   */
  virtual void publishData(const LogElementBase<TimestampPair> & time , unsigned int noCollectDataCalls) = 0;

  /** Reads buffer and writes data to a file
   *  @param time                 vector of times
   *  @param noCollectDataCalls   number of collectLoggerData calls, this allows time synchronization of the log file
   *  @param type                 type of the log file
   */
  virtual void saveDataToLogFile(const std::vector<TimestampPair> & times, unsigned int noCollectDataCalls, LogFileType type = LogFileType::BINARY) = 0;

  //! Stores a copy of the current buffer, file is saved from this
  virtual void createLocalBufferCopy() = 0;

  //! Reset logger element called before logger start
  virtual void restartElement() = 0;

  //! Cleanup logger element
  virtual void cleanupElement() = 0;

  //! @return flag indicating if log element is enabled
  virtual  bool isEnabled() const = 0;

  //! @param flag indicating if log element should be enabled
  virtual void setIsEnabled(const bool isEnabled) = 0;

  //! @return name of the log element
  virtual std::string getName() const = 0;

  //! @return unit of the log element
  virtual std::string getUnit() const = 0;

  //! @param desired unit of the log element
  virtual void setUnit(const std::string & unit) = 0;

  //! @return update frequency divider
  virtual unsigned int getDivider() const = 0;

  //! @param desired update frequency divider
  virtual void setDivider(unsigned int divider) = 0;

  //! @return check action for publishing
  virtual bool isPublished() const = 0;

  //! @return check action for saving
  virtual bool isSaved() const = 0;

  //! @return check copy action for saving
  virtual bool isCopySaved() const = 0;

  //! @return action log element takes
  virtual LogElementAction getAction() const = 0;

  //! @param desired action log element takes
  virtual void setAction(const LogElementAction action) = 0;

  //! @return buffer size of the log element
  virtual std::size_t getBufferSize() const = 0;

  //! @param desired buffer size of the log element
  virtual void setBufferSize(const std::size_t bufferSize) = 0;

  //! @return type of the buffer
  virtual BufferType getBufferType() const = 0;

  //! @param desired type of the buffer
  virtual void setBufferType(const BufferType bufferType) = 0;

  //! @return number of total items in buffer
  virtual std::size_t noItemsInBuffer() const = 0;

  //! @return number of unread items in buffer
  virtual std::size_t noUnreadItemsInBuffer() const = 0;

  //! Clear Buffer
  virtual void clearBuffer() = 0;

  //! @return mutex of the log element
  virtual std::mutex& acquireMutex() const = 0;

};

} /* namespace signal_logger */
