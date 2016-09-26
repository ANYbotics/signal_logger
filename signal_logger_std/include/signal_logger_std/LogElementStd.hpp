/*
 * BufferInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: gabrielhottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementBase.hpp"
#include "signal_logger_std/signal_logger_std_traits.hpp"

#include <fstream>

namespace signal_logger_std {

template <typename ValueType_, typename Enable_ = void>
class LogElementStd: public signal_logger::LogElementBase<ValueType_>
{

 public:
  LogElementStd(ValueType_ * ptr, std::string name, std::size_t buffer_size, std::ofstream * file) :
    signal_logger::LogElementBase<ValueType_>(ptr, name, buffer_size),
    file_(file)
  {

  }

  virtual ~LogElementStd() {

  }

  void prepareFile() {
    signal_logger_std::traits::sls_traits<ValueType_>::writeHeader(file_, this->name_);
  }

  void publish() {
    ValueType_ * ptr = new ValueType_();
    signal_logger::LogElementBase<ValueType_>::readBuffer(ptr);
    signal_logger_std::traits::sls_traits<ValueType_>::writeToFile(file_, ptr);
  }

 protected:
  std::ofstream * file_;


};


template <typename ValueType_>
class LogElementStd<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type> : public signal_logger::LogElementBase<ValueType_>
{

 public:
  LogElementStd(ValueType_ * ptr, std::string name, std::size_t buffer_size, std::ofstream * file) :
    signal_logger::LogElementBase<ValueType_>(ptr, name, buffer_size),
    file_(file)
  {

  }

  virtual ~LogElementStd() {

  }

  void prepareFile() {
    signal_logger_std::traits::sls_traits<ValueType_>::writeHeader(file_, this->name_, this->no_rows_, this->no_cols_);
  }

  void publish() {
    ValueType_ * ptr = new ValueType_();
    signal_logger::LogElementBase<ValueType_>::readBuffer(ptr);
    signal_logger_std::traits::sls_traits<ValueType_>::writeToFile(file_, ptr);
  }

 protected:
  std::ofstream * file_;


};


} /* namespace signal_logger */

