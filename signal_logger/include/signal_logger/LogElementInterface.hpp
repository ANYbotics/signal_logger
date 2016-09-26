/*
 * LogElementInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Signal logger
#include "signal_logger/BufferInterface.hpp"
#include "signal_logger/Buffer.hpp"

namespace signal_logger {

//! A class that defines the basic interface of logger elements.
/**
 *  A list of pointers to this class can be stored in the logger, since the class itself is not templated.
 */
class LogElementInterface
{
 protected:
  /** Constructor
   * @param pBuffer pointer to buffer that stores entries of ValueType_
   * @param type    typeindex of ValueType_ / datatype id of the log element
   * @param name    name of the log element
   * @param unit    unit of the logged data
   * @return log element
  */
  LogElementInterface(internal::BufferInterface * pBuffer,
                      const std::type_index & type,
                      const std::string & name,
                      const std::string & unit) :
    pBuffer_(pBuffer),
    type_(type),
    name_(name),
    unit_(unit)
  {

  }

 public:
  //! Destructor
  virtual ~LogElementInterface()
  {

  }

  //! Interface that all Log Elements shall provide

  //! Reads pointer and pushes the data into the buffer
  virtual void collectData() = 0;

  //! Reads buffer and processes data (called at every timestep)
  virtual void publishData() = 0;

  //! Writes the header of the log File
  virtual void writeHeaderToLogFile() = 0;

  //! Reads one item from the buffer and writes it to the log file. (called on storage of log files)
  virtual void writeDataToLogFile() = 0;

  /** Function to push an item to the buffer. If the type added mismatches the buffer type and error will be thrown.
   * @tparam ValueType_ Type of the value to add
   * @param  item       item of type ValueType_ that shall be added
  */
  template<typename ValueType_>
  void push_front(typename boost::call_traits<ValueType_>::param_type item) {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<Buffer<ValueType_> >(pBuffer_)->push_front(item);
    }
    else {
      throw std::runtime_error("Buffer value type mismatch");
    }
  }

  /** Function to pop an item from the buffer. If the pop data type mismatches the buffer type and error will be thrown.
   * @tparam ValueType_ Type of the value to add
   * @param  pItem      pointer to the an object where the popped data shall be stored in
  */
  template<typename ValueType_>
  void pop_back(ValueType_* pItem) {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<Buffer<ValueType_> >(pBuffer_)->pop_back(pItem);
    }
    else {
      throw std::runtime_error("Buffer value type mismatch");
    }
  }

  //! @return name of the log element
  std::type_index getType() {
    return type_;
  }

  //! @return name of the log element
  std::string getName() {
    return name_;
  }

  //! @return unit of the log element
  std::string getUnit() {
    return unit_;
  }

 private:
  internal::BufferInterfacePtr pBuffer_;
  std::type_index type_;
  std::string name_;
  std::string unit_;

};

} /* namespace signal_logger */

