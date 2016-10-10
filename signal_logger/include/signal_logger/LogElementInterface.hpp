/*
 * LogElementInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/LogElementTypes.hpp"

// STL
#include <string>

namespace signal_logger {

template <typename ValueType_>
class LogElementBase;

//! A class that defines the basic interface of logger elements.
/**
 *  A list of pointers to this class can be stored in the logger, since the class itself is not templated.
 */
class LogElementInterface
{
 public:
  //! Enum containing possible logging actions
  enum class LogElementAction: unsigned int
  {
    SAVE_AND_PUBLISH = 0,
    SAVE = 1,
    PUBLISH = 2
  };

 public:
  /** Constructor
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   *  @param bufferSize size of the buffer (bufferSize elementes of type ValueType_)
   *  @param isBufferLooping is the buffer replacing old values with new ones?
   */
  LogElementInterface(const std::string & name,
                      const std::string & unit,
                      const std::size_t divider,
                      const LogElementAction action,
                      const std::size_t bufferSize,
                      const bool isBufferLooping) :
      name_(name),
      unit_(unit),
      divider_(divider),
      action_(action),
      bufferSize_(bufferSize),
      isBufferLooping_(isBufferLooping),
      isEnabled_(false)
  {

  }

  //! Destructor
  virtual ~LogElementInterface() { }

  //! Reads pointer and pushes the data into the buffer
  virtual void collectData() = 0;

  //! Reads buffer and processes data (called at every timestep)
  virtual void publishData(signal_logger::LogElementBase<TimestampPair> * time ) = 0;

  //! Write header of log file
  virtual void saveDataToLogFile() = 0;

  //! Initialize logger elements communication etc
  virtual void initializeElement() = 0;

  //! Shutdown logger elements communication etc
  virtual void shutdownElement() = 0;

  //! @return name of the log element
  std::string getName() const {
    return name_;
  }

  //! @return unit of the log element
  std::string getUnit() const {
    return unit_;
  }

  //! @param desired unit of the log element
  void setUnit(const std::string & unit) {
    unit_ = unit;
  }

  //! @return update frequency divider
  unsigned int getDivider() const {
    return divider_;
  }

  //! @param desired update frequency divider
  void setDivider(unsigned int divider) {
    divider_ = divider;
  }

  //! @return check action for publishing
  bool isPublished() {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::PUBLISH);
  }

  //! @return check action for saving
  bool isSaved() {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::SAVE);
  }

  //! @return action log element takes
  LogElementAction getAction() const {
    return action_;
  }

  //! @param desired action log element takes
  void setAction(LogElementAction action) {
    action_ = action;
  }

  //! @return buffer size of the log element
  virtual std::size_t getBufferSize() const {
    return bufferSize_;
  }

  //! @param desired buffer size of the log element
  virtual void setBufferSize(const std::size_t bufferSize) = 0;

  //! @return flag indicating if buffer is looping
  virtual bool isBufferLooping() const {
    return isBufferLooping_;
  }

  //! @param flag indicating if buffer should be looping
  virtual void setIsBufferLooping(const bool isBufferLooping) = 0;

  //! @return number of total items in buffer
  virtual std::size_t noItemsInBuffer() const = 0;
  //! @return number of unread items in buffer
  virtual std::size_t noUnreadItemsInBuffer() const = 0;
  //! Clear Buffer
  virtual void clearBuffer() = 0;


  //! @return flag indicating if log element is enabled
  bool isEnabled() const {
    return isEnabled_;
  }

  //! @param flag indicating if log element should be enabled
  void setIsEnabled(const bool isEnabled) {
    if(isEnabled != isEnabled_)
    {
      isEnabled_ = isEnabled;
      if(isEnabled_) {
        this->setBufferSize(bufferSize_);
        this->initializeElement();
      }
      else {
        this->shutdownElement();
        this->setBufferSize(std::size_t(0));
      }
    }
  }

 protected:
  //! Name of the log element
  std::string name_;
  //! Unit of the log element
  std::string unit_;
  //! Defines log element collection frequency = updateFrequency/divider
  std::size_t divider_;
  //! Action
  LogElementAction action_;
  //! Buffer size
  std::size_t bufferSize_;
  //! Buffer looping
  bool isBufferLooping_;
  //! Indicates if log element is currently active
  bool isEnabled_;
};

} /* namespace signal_logger */

