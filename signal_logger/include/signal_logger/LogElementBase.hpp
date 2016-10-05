/*
 * LogElementBase.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementInterface.hpp"
#include "signal_logger/Buffer.hpp"

namespace signal_logger {

template <typename ValueType_>
class LogElementBase: public LogElementInterface
{
 public:
  /** Constructor
   *  @param ptr        pointer to the log var
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   *  @param bufferSize size of the buffer (bufferSize elementes of type ValueType_)
   *  @param isBufferLooping is the buffer replacing old values with new ones?
   */
  LogElementBase(ValueType_ * ptr,
                 const std::string & name,
                 const std::string & unit,
                 const std::size_t divider,
                 const LogElementAction action,
                 const std::size_t bufferSize,
                 const bool isBufferLooping) :
    LogElementInterface(name, unit, divider, action, bufferSize, isBufferLooping),
    buffer_(ptr, 0) // Zero buffer size log element not enabled
  {

  }

  //! Destructor
  virtual ~LogElementBase()
  {

  }

  //! Push data to the buffer
  void collectData()
  {
    buffer_.collect();
  }

  //! @param desired buffer size of the log element
  virtual void setBufferSize(const std::size_t bufferSize) {
    bufferSize_ = bufferSize;
    buffer_.setBufferSize(bufferSize_);
  }

  //! @param flag indicating if buffer should be looping
  virtual void setIsBufferLooping(const bool isBufferLooping) {
    isBufferLooping_ = isBufferLooping;
  }

  //! @return flag indicating if buffer is full
  virtual bool isBufferFull() const {
    return buffer_.isLooping() ? bufferSize_ <= buffer_.noUnreadItems() : bufferSize_ <= buffer_.noItems();
  }

  //! @return all read and unread elements in the buffer
  std::vector<ValueType_> getBufferCopy() {
    return buffer_.copyBuffer();
  }

 protected:
  Buffer<ValueType_> buffer_;
};

} /* namespace signal_logger */
