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
#include <ctime>
#include <ratio>
#include <chrono>

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
  virtual bool workerSaveData(const std::string & logFileName) {

    // Open file
    file_.open(logFileName, std::ios::out | std::ios::trunc);
    for(auto & elem : logElements_) {
      if(elem.second->isCollected()) {
        elem.second->writeHeaderToLogFile();
      }
    }
    //    # FIXME for binary
    //    file_.close();
    //    file_.open(filename, std::ios::out | std::ios::app | std::ios::binary);
    for(auto & elem : logElements_) {
      if(elem.second->isCollected()) {
        elem.second->writeDataToLogFile();
      }
    }
    file_<<std::endl;
    for(auto & elem : logElements_) {
      if(elem.second->isCollected()) {
        elem.second->writeDataToLogFile();
      }
    }

    file_.close();

    MELO_INFO( "All done, captain!" );

    return true;

  }

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeStd; }

 protected:
  FOR_ALL_TYPES(ADD_VAR_IMPLEMENTATION)

 private:
 std::ofstream file_;
};

} /* namespace signal_logger */
