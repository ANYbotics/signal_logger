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
   * @param pBuffer pointer to buffer that stores entries of ValueType_ (Length is zero until not collected)
   * @param type    typeindex of ValueType_ / datatype id of the log element
   * @param name    name of the log element
   * @param unit    unit of the logged data
   * @return log element
   */
  LogElementInterface(internal::BufferInterface * pBuffer,
                      const std::size_t & bufferSize,
                      const std::type_index & type,
                      const std::string & name,
                      const std::string & unit,
                      const unsigned int divider) :
                        pBuffer_(pBuffer),
                        bufferSize_(bufferSize),
                        type_(type),
                        name_(name),
                        unit_(unit),
                        divider_(divider),
                        isEnabled_(false)
 {

 }

 public:
  //! Destructor
  virtual ~LogElementInterface() { }

  //! Reads pointer and pushes the data into the buffer
  virtual void collectData() = 0;

  //! Reads buffer and processes data (called at every timestep)
  virtual void publishData() = 0;

  //! Write header of log file
  virtual void writeHeaderToLogFile() = 0;

  //! Write data to log file
  virtual void writeDataToLogFile() = 0;

  //! Initialize logger elements communication etc
  virtual void initialize() = 0;

  //! Shutdown logger elements communication etc
  virtual void shutdown() = 0;

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

  /** Function to pop all items from the buffer. If the pop data type mismatches the buffer type and error will be thrown.
   *  However unread flag is untouched. This basically is a copy of the buffer entries, but they remain unchanged.
   * @tparam ValueType_ Type of the value to read
   * @return  all data stored in the buffer
   */
  template<typename ValueType_>
  std::vector<ValueType_> read_full_buffer() {
    if (type_ == typeid(ValueType_)) {
      return std::static_pointer_cast<Buffer<ValueType_> >(pBuffer_)->read_full_buffer();
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
  std::string getName() const {
    return name_;
  }

  //! @return unit of the log element
  std::string getUnit() const {
    return unit_;
  }

  //! @return get update frequency divider
  unsigned int getDivider() const {
    return divider_;
  }
  //! @return whether log element is enabled
  bool isEnabled() const {
    return isEnabled_;
  }

  //! @return whether log element is enabled
  void setIsEnabled(const bool isEnabled) {
    if(isEnabled != isEnabled_)
    {
      isEnabled_ = isEnabled;
      if(isEnabled_) {
        pBuffer_->set_capacity(bufferSize_);
        this->initialize();
      }
      else {
        pBuffer_->set_capacity(0);
        this->shutdown();
      }
    }
  }

 protected:
  //! Data Buffer
  internal::BufferInterfacePtr pBuffer_;

 private:
  //! Size of the data buffer
  std::size_t bufferSize_;
  //! Type stored in the buffer
  std::type_index type_;
  //! Name of the log element
  std::string name_;
  //! Unit of the log element
  std::string unit_;
  //! Defines log element collection frequency = updateFrequency/divider
  unsigned int divider_;
  //! Indicates if log element is currently active
  bool isEnabled_;

};

} /* namespace signal_logger */

