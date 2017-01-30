/*!
 * @file	  SignalLoggerOptions.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan 30, 2017
 */

#pragma once

// signal logger
#include "signal_logger_core/typedefs.hpp"

// STL
#include <string>

namespace signal_logger {

//! Struct containing logger options
struct SignalLoggerOptions {

  //! Constructor
  SignalLoggerOptions(const unsigned int updateFrequency,
                      const double maxLoggingTime,
                      const std::string& collectScriptFileName,
                      const std::string& loggerPrefix)
  : updateFrequency_(updateFrequency),
    maxLoggingTime_(maxLoggingTime),
    collectScriptFileName_(collectScriptFileName),
    loggerPrefix_(loggerPrefix)
  {

  }

  //! Default Constructor
  SignalLoggerOptions()
  : SignalLoggerOptions(0u, LOGGER_DEFAULT_MAXIMUM_LOG_TIME, LOGGER_DEFAULT_SCRIPT_FILENAME, LOGGER_DEFAULT_PREFIX)
  {

  }

//  //! Overload equals operator
//  SignalLoggerOptions& operator=(const SignalLoggerOptions& other)
//  {
//    // Leave mutex as is!
//    this->updateFrequency_ = other.updateFrequency_;
//    this->maxLoggingTime_ = other.maxLoggingTime_;
//    this->collectScriptFileName_ = other.collectScriptFileName_;
//    this->loggerPrefix_ = other.loggerPrefix_;
//    return *this;
//  }

  //! Rate at which collectLoggerData() is called
  unsigned int updateFrequency_;
  //! Logging length
  double maxLoggingTime_;
  //! Collected data script filename
  std::string collectScriptFileName_;
  //! Logger prefix
  std::string loggerPrefix_;
};

}
