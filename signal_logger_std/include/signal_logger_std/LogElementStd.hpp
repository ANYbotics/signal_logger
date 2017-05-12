/*!
 * @file     LogElementStd.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    Implementation of a Log element for std logging. Save data to binary file.
 */

#pragma once

// Signal logger
#include "signal_logger_core/LogElementBase.hpp"
#include "signal_logger_std/signal_logger_std_traits.hpp"

// STL
#include <fstream>

namespace signal_logger_std {

template <typename ValueType_>
class LogElementStd: public signal_logger::LogElementBase<ValueType_>
{
 public:
  /** Constructor
   *  @param ptr        pointer to the log var
   *  @param bufferType buffer type of the log var
   *  @param bufferSize buffer size of the log var
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   *  @param headerStream string stream for log file header
   *  @param dataStream   type of the buffer
   */
  LogElementStd(const ValueType_ * const ptr,
                const signal_logger::BufferType bufferType,
                const std::size_t bufferSize,
                const std::string & name,
                const std::string & unit,
                const std::size_t divider,
                const signal_logger::LogElementAction action,
                std::stringstream * headerStream,
                std::stringstream * dataStream) :
      signal_logger::LogElementBase<ValueType_>(ptr, bufferType, bufferSize, name, unit, divider, action),
      headerStream_(headerStream),
      dataStream_(dataStream)
  {

  }

  //! Destructor
  virtual ~LogElementStd()
  {

  }

  //! Save Data to file
  void saveDataToLogFile(const signal_logger::TimeElement & times,
                         unsigned int nrCollectDataCalls,
                         signal_logger::LogFileType type = signal_logger::LogFileType::BINARY) override
  {
    if(type == signal_logger::LogFileType::BINARY) {
      // Lock the copy mutex
      std::unique_lock<std::mutex> lock(this->mutexCopy_);

      // Write to file
      if(this->bufferCopy_.noTotalItems() > 0 ) {
        signal_logger_std::traits::sls_traits<ValueType_, ValueType_>::writeLogElementToStreams(
            headerStream_, dataStream_, this->bufferCopy_, this->optionsCopy_.getName(), this->optionsCopy_.getDivider(),
            this->bufferCopy_.getBufferType() == signal_logger::BufferType::LOOPING);
      }
    }
  }

 protected:
  //! Header stream
  std::stringstream* headerStream_;
  //! Data stream
  std::stringstream* dataStream_;

};

} /* namespace signal_logger */
