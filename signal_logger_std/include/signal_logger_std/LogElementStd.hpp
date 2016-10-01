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
  /** Constructor
   * @param ptr     pointer to the data that shall be logged
   * @param name    name of the log element
   * @param unit    unit of the logged data
   * @param buffer_size size of the buffer (old elements will always be overwritten)
   * @param file    pointer to the log file
   * @return log element
  */
  LogElementStd(ValueType_ * ptr,
                const std::string & name,
                const std::string & unit,
                const unsigned int divider,
                const std::size_t buffer_size,
                std::ofstream * file) :
    signal_logger::LogElementBase<ValueType_>(ptr, name, unit, divider, buffer_size),
    file_(file)
  {

  }

  //! Destructor
  virtual ~LogElementStd()
  {

  }

  void writeHeaderToLogFile()
  {
    generateHeader();
  }

  // Version for not eigen matrices
  template<typename V = ValueType_>
  typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  generateHeader() {
    signal_logger_std::traits::sls_traits<V>::writeHeader(file_, this->getName(), this->pBuffer_->get_no_items(), this->getDivider());
  }

  // Version for eigen matrices
  template<typename V = ValueType_>
  typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  generateHeader() {
    signal_logger_std::traits::sls_traits<V>::writeHeader(file_, this->getName(), this->pBuffer_->get_no_items(),
                                                          this->getDivider(), this->rows(), this->cols());
  }

  //! Write Data
  virtual void writeDataToLogFile() {
    writeData();
  }

  // Version for not eigen matrices
  template<typename V = ValueType_>
  typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  writeData() {
    std::vector<ValueType_> data = signal_logger::LogElementInterface::read_full_buffer<ValueType_>();
    signal_logger_std::traits::sls_traits<V>::writeToFile(file_, data);
  }

  // Version for eigen matrices
  template<typename V = ValueType_>
  typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  writeData() {
    std::vector<typename ValueType_::Scalar> data = signal_logger::LogElementInterface::read_full_buffer<typename ValueType_::Scalar>();
    signal_logger_std::traits::sls_traits<V>::writeToFile(file_, data, this->rows(), this->cols());
  }


  //! Publish no implementation
  virtual void publishData() { }

  //! Initizalize -> empty
  virtual void initialize() { }

  //! Shutdown -> empty
  virtual void shutdown() { }

 protected:
  std::ofstream * file_;


};

} /* namespace signal_logger */

