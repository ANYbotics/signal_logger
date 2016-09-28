/*
 * SignalLoggerStd.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <signal_logger_std/LogElementStd.hpp>
#include "signal_logger_std/macro_definitions.hpp"
#include "signal_logger/macro_definitions.hpp"

#include "signal_logger/SignalLoggerBase.hpp"

namespace signal_logger_std {

class SignalLoggerStd : public signal_logger::SignalLoggerBase
{
 public:
  SignalLoggerStd():
    signal_logger::SignalLoggerBase(),
    file_()
  {

  }

  virtual ~SignalLoggerStd()
  {

  }

  //! Save all the buffered data into a log file
  virtual void saveLoggerData() {
    std::string filename = "mydata3.txt";
    file_.open(filename, std::ios::out | std::ios::trunc);
    file_ << "Begin file"<<std::endl;
    for(auto & elem : logElements_) {
      elem.second->writeHeaderToLogFile();
    }
    //    # FIXME for binary
    //    file_.close();
    //    file_.open(filename, std::ios::out | std::ios::app | std::ios::binary);
    for(auto & elem : logElements_) {
      elem.second->writeDataToLogFile();
    }
    file_<<std::endl;
    for(auto & elem : logElements_) {
      elem.second->writeDataToLogFile();
    }
    file_.close();

  }

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeStd; }

 protected:
  FOR_ALL_TYPES(ADD_VAR_IMPLEMENTATION)

 private:
 std::ofstream file_;
};

} /* namespace signal_logger */
