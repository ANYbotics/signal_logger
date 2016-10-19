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
  SignalLoggerStd(const std::string & loggerPrefix = std::string{signal_logger::SignalLoggerBase::LOGGER_DEFAULT_PREFIX});
  virtual ~SignalLoggerStd();

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName);

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeStd; }

  /** Resets the pointer to the logelement
   * @param buffertype type of the time buffer
   * @param maxLogTime maximal time logging
   */
  virtual bool resetTimeLogElement(signal_logger::BufferType buffertype,
                                   double maxLogTime = signal_logger::SignalLoggerBase::LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME) override;


 protected:
  FOR_ALL_TYPES(ADD_STD_VAR_IMPLEMENTATION);

 protected:
  std::ofstream file_;
  std::stringstream headerStream_;
  std::stringstream dataStream_;
};

} /* namespace signal_logger_std */
