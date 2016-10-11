/*
 * LogElementStd.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementBase.hpp"
#include "signal_logger_std/signal_logger_std_traits.hpp"

// STL
#include <fstream>

namespace signal_logger_std {

template <typename ValueType_>
class LogElementStd: public signal_logger::LogElementBase<ValueType_>
{
 public:
  LogElementStd(ValueType_ * ptr,
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
  void saveDataToLogFile() override
  {
    std::vector<ValueType_> values = this->buffer_.copyBuffer();
    signal_logger_std::traits::sls_traits<ValueType_>::writeLogElementToStreams(headerStream_, dataStream_, values, this->getName(), this->getDivider());
  }

 protected:
  std::stringstream* headerStream_;
  std::stringstream* dataStream_;

};

} /* namespace signal_logger */

