/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Gabriel Hottiger
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
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

// STL
#include <string>
#include <mutex>

namespace signal_logger {

//! Enum containing possible logging actions
enum class LogElementAction: unsigned int
{
  NONE = 0,/*!< 0 */
  SAVE_AND_PUBLISH = 1,/*!< 1 */
  SAVE = 2,/*!< 2 */
  PUBLISH = 3/*!< 3 */
};

//! Enum containing possible log file types
enum class LogFileType: unsigned int
{
  BINARY = 0,/*!< 0 */
  BAG = 1, /*!< 1 */
  BINARY_AND_BAG = 2/*!< 2 */
};

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
