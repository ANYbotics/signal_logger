/*!
 * @file     SignalLoggerNone.hpp
 * @author   Gabriel Hottiger
 * @date     Oct 10, 2016
 * @brief    Empty implementation of the signal logger
 */

#pragma once

// signal logger
#include "signal_logger_core/SignalLoggerBase.hpp"

namespace signal_logger {

class SignalLoggerNone : public SignalLoggerBase
{
 public:
  //! Constructor
  SignalLoggerNone() { }

  //! Destructor
  virtual ~SignalLoggerNone() { }

  //! Add
  template<typename ValueType_>
  void add( const ValueType_ * const var,
            const std::string & name,
            const std::string & group       = SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit        = SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider       = SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
            const LogElementAction action   = SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize    = SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const BufferType bufferType     = SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
  {
  }

  //! @copydoc SignalLoggerBase::initLogger
  virtual void initLogger(const SignalLoggerOptions& options) override { }

  //! @copydoc SignalLoggerBase::startLogger
  virtual bool startLogger(bool updateLogger = false) override { return true; }

  //! @copydoc SignalLoggerBase::stopLogger
  virtual bool stopLogger() override { return true; }

  //! @copydoc SignalLoggerBase::restartLogger
  virtual bool restartLogger(bool updateLogger = false) override { return true; }

  //! @copydoc SignalLoggerBase::updateLogger
  virtual bool updateLogger(const bool readScript = true, const std::string & scriptname = "") override { return true; }

  //! @copydoc SignalLoggerBase::saveLoggerScript
  virtual bool saveLoggerScript(const std::string & scriptName = SignalLoggerOptions::LOGGER_DEFAULT_SCRIPT_FILENAME) override { return true; }

  //! @copydoc SignalLoggerBase::collectLoggerData
  virtual bool collectLoggerData() override { return true; }

  //! @copydoc SignalLoggerBase::publishData
  virtual bool publishData() override { return true; }

  //! @copydoc SignalLoggerBase::saveLoggerData
  virtual bool saveLoggerData(const signal_logger::LogFileTypeSet & logfileTypes) override { return true; }

  //! @copydoc SignalLoggerBase::stopAndSaveLoggerData
  virtual bool stopAndSaveLoggerData(const signal_logger::LogFileTypeSet & logfileTypes) override { return true; }

 protected:
  //! @copydoc SignalLoggerBase::workerSaveData
  virtual bool workerSaveData(const std::string & logFileName, const std::string & pathWithPrefix, const signal_logger::LogFileTypeSet & logfileTypes) override { return false; };

};

}
