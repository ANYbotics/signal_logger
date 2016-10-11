/*
 * SignalLoggerStd.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger_std/macro_definitions.hpp"
#include "signal_logger/SignalLoggerBase.hpp"

// STL
#include <fstream>
#include <sstream>
#include <memory>

namespace signal_logger_std {

class SignalLoggerStd : public signal_logger::SignalLoggerBase
{
 public:
  SignalLoggerStd(const std::string & loggerPrefix = signal_logger::LOGGER_DEFAULT_PREFIX);
  virtual ~SignalLoggerStd();

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName);

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeStd; }

 protected:
  FOR_ALL_TYPES(ADD_STD_VAR_IMPLEMENTATION);

 protected:
  std::ofstream file_;
  std::stringstream headerStream_;
  std::stringstream dataStream_;
};

} /* namespace signal_logger_std */
