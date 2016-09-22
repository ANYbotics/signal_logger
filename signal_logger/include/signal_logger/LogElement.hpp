/*
 * BufferInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: gabrielhottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementInterface.hpp"

namespace signal_logger {

template <typename ValueType_>
class LogElement: public LogElementInterface
{
 public:
  LogElement(ValueType_ * ptr, std::string name, std::size_t buffer_size) :
    LogElementInterface(typeid(ValueType_), name),
    ptr_(ptr)
  {
    pBuffer_ = new Buffer<ValueType_>(buffer_size);
  }

  virtual ~LogElement() {

  }

  void collect() {
    this->push_front(*ptr_);
  }

 protected:
  ValueType_ * ptr_;

};

} /* namespace signal_logger */

