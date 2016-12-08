/*
 * LogElementStd.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
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
   *  @param ptr          pointer to the log var
   *  @param name         name of the log var
   *  @param unit         unit of the log var
   *  @param divider      log_freq = ctrl_freq/divider
   *  @param action       save, publish or save and publish
   *  @param bufferSize   size of the buffer (bufferSize elements of type ValueType_)
   *  @param bufferType   type of the buffer
   *  @param headerStream string stream for log file header
   *  @param dataStream   type of the buffer
   */
  LogElementStd(const ValueType_ * const ptr,
                const std::string & name,
                const std::string & unit,
                const std::size_t divider,
                const signal_logger::LogElementAction action,
                const std::size_t bufferSize,
                const signal_logger::BufferType bufferType,
                std::stringstream * headerStream,
                std::stringstream * dataStream) :
      signal_logger::LogElementBase<ValueType_>(ptr, name, unit, divider, action, bufferSize, bufferType),
      headerStream_(headerStream),
      dataStream_(dataStream)
  {

  }

  //! Destructor
  virtual ~LogElementStd()
  {

  }

  //! Save Data to file
  void saveDataToLogFile(const signal_logger::LogElementBase<signal_logger::TimestampPair> & time,
                         unsigned int nrCollectDataCalls,
                         signal_logger::LogFileType type = signal_logger::LogFileType::BINARY) override
  {
    if(type == signal_logger::LogFileType::BINARY) {
      std::vector<ValueType_> values = this->buffer_.copyBuffer();
      signal_logger_std::traits::sls_traits<ValueType_>::writeLogElementToStreams(
          headerStream_, dataStream_, values, this->getName(), this->getDivider(),
          this->getBufferType() == signal_logger::BufferType::LOOPING);
    }
  }

 protected:
  //! Header stream
  std::stringstream* headerStream_;
  //! Data stream
  std::stringstream* dataStream_;

};

} /* namespace signal_logger */
