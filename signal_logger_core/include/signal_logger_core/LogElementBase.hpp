/*!
 * @file     LogElementBase.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    Templated base class for log elements.
 */

#pragma once

// Signal logger
#include "signal_logger_core/LogElementInterface.hpp"
#include "signal_logger_core/Buffer.hpp"

// STL
#include <atomic>
#include <mutex>

namespace signal_logger {

template <typename ValueType_>
class LogElementBase: public LogElementInterface
{
 public:
  /** Constructor
   *  @param ptr        pointer to the log var
   *  @param options    options of the log var
   */
  LogElementBase(const ValueType_ * const ptr,
                 const BufferType  bufferType,
                 const std::size_t bufferSize,
                 const LogElementOptions & options) :
   LogElementInterface(),
   buffer_(ptr), // Zero buffer size log element not enabled
   options_(options),
   mutex_(),
   bufferCopy_(),
   optionsCopy_(),
   mutexCopy_()
 {
    // Initialize the buffer with the given options
    buffer_.setType(bufferType);
    buffer_.setBufferSize(bufferSize);
 }

  //! Destructor
  virtual ~LogElementBase() { }

  //! Data collection is not element specific
  void collectData() override final { buffer_.collect(); }

  //! Reads buffer and processes data (probably called from different thread)
  virtual void publishData(const TimeElement & time,
                           unsigned int nrCollectDataCalls) override { }

  //! Writes local buffer copy to a file
  virtual void saveDataToLogFile(const TimeElement & times,
                                 unsigned int nrCollectDataCalls,
                                 LogFileType type = LogFileType::BINARY) override { }

  //! Stores a copy of the current buffer, file is saved from this
  virtual void copy() {
    // Lock all mutexes and copy the buffer
    std::unique_lock<std::mutex> lock(mutex_);
    std::unique_lock<std::mutex> lockCopy(mutexCopy_);
    bufferCopy_ = std::move(buffer_);
    optionsCopy_ = options_;
  }

  //! Reset logger element called before logger start
  virtual void reset() override { buffer_.clear(); }

  //! Cleanup logger element
  virtual void cleanup() override { }

  //! @return options of the log element
  LogElementOptions & getOptions() final { return options_; }

  //! @return buffer of the log element
  const BufferInterface & getBuffer() const final { return buffer_; }

  //! @return non const buffer interface
  BufferInterface & getBuffer() { return buffer_; }


  //! @return mutex of the log element
  std::mutex& acquireMutex() const { return mutex_; }

  /*** Get the timestamp at position in the buffer
   *   @tparam V  log element type (ValueType_)
   *   @param  n  position in the buffer
   *   @return    Timestamp-pair at position n in buffer
   */
  template<typename V = ValueType_>
  V getTimeStampAtPosition(std::size_t n, typename std::enable_if<std::is_same<TimestampPair, V>::value>::type* = 0 /* is timestamp pair */) const
  {
    return buffer_.readElementAtPosition(n);
  }

  /*** Get the timestamp at position in the buffer
   *   @tparam V  log element type (ValueType_)
   *   @return    buffer copy
   */
  template<typename V = ValueType_>
  const signal_logger::vector_type<V> & getTimeBufferCopy(typename std::enable_if<std::is_same<TimestampPair, V>::value>::type* = 0 /* is timestamp pair */) const
  {
    std::unique_lock<std::mutex> lock(mutexCopy_);
    return bufferCopy_;
  }

 protected:
  //! Update the element
  virtual void updateElement() { };

 protected:
  //! Buffer (threadsafe)
  Buffer<ValueType_> buffer_;
  //! Options of the log element
  LogElementOptions options_;
  //! Mutex protecting element
  std::mutex mutex_;

  //! Buffer copy (actually the buffer is moved during saving)
  Buffer<ValueType_> bufferCopy_;
  //! Options copy of the log element
  LogElementOptions optionsCopy_;
  //! Mutex protecting element copy
  std::mutex mutexCopy_;

};

} /* namespace signal_logger */
