/*
 * LogElementBase.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Signal logger
#include "signal_logger_core/LogElementInterface.hpp"
#include "signal_logger_core/Buffer.hpp"

// STL
#include <atomic>

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
   *  @param bufferSize size of the buffer (bufferSize elements of type ValueType_)
   *  @param bufferType type of the buffer
   */
  LogElementBase(const ValueType_ * const ptr,
                 const std::string & name,
                 const std::string & unit,
                 const std::size_t divider,
                 const LogElementAction action,
                 const std::size_t bufferSize,
                 const BufferType bufferType) :
                   LogElementInterface(),
                   buffer_(ptr), // Zero buffer size log element not enabled
                   name_(name),
                   unit_(unit),
                   divider_(divider),
                   action_(action),
                   isEnabled_(false),
                   mutex_(),
                   bufferCopy_(),
                   nameCopy_(),
                   dividerCopy_(),
                   isBufferLoopingCopy_(),
                   copyMutex_()
 {
    buffer_.setType(bufferType);
    buffer_.setBufferSize(bufferSize);
    buffer_.clear();
 }

  //! Destructor
  virtual ~LogElementBase() { }

  //! Data collection is not element specific
  void collectData() override final { buffer_.collect(); }

  //! Reads buffer and processes data (probably called from different thread)
  virtual void publishData(const LogElementBase<TimestampPair> & time, unsigned int nrCollectDataCalls) override { }

  //! Writes local buffer copy to a file
  virtual void saveDataToLogFile(const std::vector<TimestampPair> & times, unsigned int nrCollectDataCalls, LogFileType type = LogFileType::BINARY) override {}

  //! Stores a copy of the current buffer, file is saved from this
  virtual void createLocalBufferCopy() {
    // Lock all mutexes and copy the buffer
    std::unique_lock<std::mutex> lock(mutex_);
    std::unique_lock<std::mutex> lockCopy(copyMutex_);
    bufferCopy_ = buffer_.copyBuffer();
    nameCopy_ = name_;
    dividerCopy_ = divider_.load();
    actionCopy_ = action_.load();
    isBufferLoopingCopy_ = (buffer_.getType() == BufferType::LOOPING);
  }

  //! Reset logger element called before logger start
  virtual void restartElement()    override {
    this->clearBuffer();
  }

  //! Cleanup logger element
  virtual void cleanupElement()    override { }

  //! @return flag indicating if log element is enabled
  bool isEnabled() const final {
    return isEnabled_;
  }

  //! @param flag indicating if log element should be enabled
  void setIsEnabled(const bool isEnabled) override final {
    isEnabled_ = isEnabled;
    buffer_.allocate(isEnabled_);
    updateElement();
  }

  //! @return name of the log element
  std::string getName() const override final {
    std::unique_lock<std::mutex> lock(mutex_);
    return name_;
  }

  //! @return unit of the log element
  std::string getUnit() const override final {
    std::unique_lock<std::mutex> lock(mutex_);
    return unit_;
  }

  //! @param desired unit of the log element
  void setUnit(const std::string & unit) override final {
    std::unique_lock<std::mutex> lock(mutex_);
    unit_ = unit;
  }

  //! @return get frequency divider
  unsigned int getDivider() const override final {
    return divider_;
  }

  //! @param desired update frequency divider
  void setDivider(unsigned int divider) override final {
    divider_ = divider;
  }

  //! @return check action for publishing
  bool isPublished() const override final {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::PUBLISH);
  }

  //! @return check action for saving
  bool isSaved() const override final {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::SAVE);
  }

  //! @return check action for saving
  bool isCopySaved() const override final {
    return (actionCopy_ == LogElementAction::SAVE_AND_PUBLISH || actionCopy_ == LogElementAction::SAVE);
  }

  //! @return action log element takes
  LogElementAction getAction() const override final {
    return action_;
  }

  //! @param desired action log element takes
  void setAction(LogElementAction action) override final {
    action_ = action;
    updateElement();
  }

  //! @return buffer size of the log element
  std::size_t getBufferSize() const override final { return buffer_.getBufferSize(); }

  //! @param desired buffer size of the log element
  void setBufferSize(const std::size_t bufferSize) override final { buffer_.setBufferSize(bufferSize); buffer_.clear(); }

  //! @return type of the buffer
  BufferType getBufferType() const override final { return buffer_.getType(); }

  //! @param desired type of the buffer
  void setBufferType(const BufferType bufferType) override final { buffer_.setType(bufferType); }

  //! @return number of total items in buffer
  std::size_t noItemsInBuffer() const override final { return buffer_.noItems(); }

  //! @return number of unread items in buffer
  std::size_t noUnreadItemsInBuffer() const override final { return buffer_.noUnreadItems(); }

  //! Clear buffer
  void clearBuffer() override final { buffer_.clear(); }

  //! @return mutex of the log element
  std::mutex& acquireMutex() const { return mutex_; }

  /*** Get the timestamp at position in the buffer
   *   @tparam V  log element type (ValueType_)
   *   @param  n  position in the buffer
   *   @return    Timestamp-pair at position n in buffer
   */
  template<typename V = ValueType_>
  V getTimeStampAtPosition(std::size_t n, typename std::enable_if<std::is_same<TimestampPair, V>::value>::type* = 0 /* is timestamp pair */) const final
  {
    return buffer_.readElementAtPosition(n);
  }

  /*** Get the timestamp at position in the buffer
   *   @tparam V  log element type (ValueType_)
   *   @return    buffer copy
   */
  template<typename V = ValueType_>
  const std::vector<V> & getTimeBufferCopy(typename std::enable_if<std::is_same<TimestampPair, V>::value>::type* = 0 /* is timestamp pair */) const final
  {
    std::unique_lock<std::mutex> lock(copyMutex_);
    return bufferCopy_;
  }

 protected:
  //! Update the element
  virtual void updateElement() { };

 protected:
  //! Buffer (threadsafe)
  Buffer<ValueType_> buffer_;
  //! Name of the log element
  std::string name_;
  //! Unit of the log element
  std::string unit_;
  //! Defines log element collection frequency = updateFrequency/divider
  std::atomic<std::size_t> divider_;
  //! Action
  std::atomic<LogElementAction> action_;
  //! Indicates if log element is currently active
  std::atomic_bool isEnabled_;
  //! Mutex
  mutable std::mutex mutex_;
  //! Local copy
  std::vector<ValueType_> bufferCopy_;
  //! Local name copy
  std::string nameCopy_;
  //! Local divider copy
  std::atomic<std::size_t> dividerCopy_;
  //! Action
  std::atomic<LogElementAction> actionCopy_;
  //! Looping buffer local copy
  std::atomic_bool isBufferLoopingCopy_;
  //! Local copy mutex
  mutable std::mutex copyMutex_;
};

} /* namespace signal_logger */
