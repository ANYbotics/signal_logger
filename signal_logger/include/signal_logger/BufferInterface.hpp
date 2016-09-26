/*
 * BufferInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <memory>

namespace signal_logger {

namespace internal {

class BufferInterface;

typedef std::shared_ptr<BufferInterface> BufferInterfacePtr;

class BufferInterface {
 public:
  BufferInterface() {

  }

  virtual ~BufferInterface() {

  }

  virtual void set_capacity(std::size_t new_capacity) = 0;

};

} // namespace internal

} /* namespace signal_logger */

