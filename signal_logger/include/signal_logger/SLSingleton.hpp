/*
 * SLSingleton.hpp
 *
 *  Created on: Sep 28, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <signal_logger/SignalLoggerBase.hpp>

namespace signal_logger {

// From http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
class SignalLogger
{
 public:
  static SignalLogger& get()
  {
    static SignalLogger    instance; // Guaranteed to be destroyed.
    // Instantiated on first use.
    return instance;
  }

 private:
  SignalLogger(): logger_(nullptr) {};

 public:
  SignalLogger(SignalLogger const&)    = delete;
  void operator=(SignalLogger const&)  = delete;

  signal_logger::SignalLoggerBase * logger()
  {
    return logger_;
  }

  void setLogger(signal_logger::SignalLoggerBase * logger)
  {
    logger_ = logger;
  }

 private:
  signal_logger::SignalLoggerBase * logger_;
};

}
