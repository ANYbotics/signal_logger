/*
 *  signal_logger.hpp
 *
 *  Created on: Sep 28, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <signal_logger/SignalLoggerBase.hpp>

namespace signal_logger {

/**
 * Adapted from: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
 */
//! Implementation of a logger Singleton
class SignalLogger
{
 public:
  //! Accessor
  static SignalLogger& get()
  {
    static SignalLogger    instance; // Lazy instantiation
    return instance;
  }

  signal_logger::SignalLoggerBase * logger()
  {
    return logger_;
  }

  void setLogger(signal_logger::SignalLoggerBase * logger)
  {
    logger_ = logger;
  }

 private:
  //! Private constructor
  SignalLogger(): logger_(nullptr) {};

 public:
  //! Delete copy constructor and = operator
  SignalLogger(SignalLogger const&)    = delete;
  void operator=(SignalLogger const&)  = delete;

 private:
  signal_logger::SignalLoggerBase * logger_;
};

}
