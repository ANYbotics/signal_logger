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
  void add( [[maybe_unused]] const ValueType_ * const var,
            [[maybe_unused]] const std::string & name,
            [[maybe_unused]] const std::string & group                      = SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
            [[maybe_unused]] const std::string & unit                       = SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
            [[maybe_unused]] const std::size_t divider                      = SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
            [[maybe_unused]] const LogElementAction action                  = SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
            [[maybe_unused]] const BufferType bufferType                    = SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE,
            [[maybe_unused]] const std::optional<std::size_t> bufferSize    = std::nullopt)
  {
  }

  //! @copydoc SignalLoggerBase::initLogger
  virtual void initLogger([[maybe_unused]] const SignalLoggerOptions& options) override { }

  //! @copydoc SignalLoggerBase::startLogger
  virtual bool startLogger([[maybe_unused]] bool updateLogger = false) override { return true; }

  //! @copydoc SignalLoggerBase::stopLogger
  virtual bool stopLogger() override { return true; }

  //! @copydoc SignalLoggerBase::restartLogger
  virtual bool restartLogger([[maybe_unused]] bool updateLogger = false) override { return true; }

  //! @copydoc SignalLoggerBase::updateLogger
  virtual bool updateLogger([[maybe_unused]] const bool readScript = true, [[maybe_unused]] const std::string & scriptname = "") override { return true; }

  //! @copydoc SignalLoggerBase::saveLoggerScript
  virtual bool saveLoggerScript([[maybe_unused]] const std::string & scriptName = SignalLoggerOptions::LOGGER_DEFAULT_SCRIPT_FILENAME) override { return true; }

  //! @copydoc SignalLoggerBase::collectLoggerData
  virtual bool collectLoggerData() override { return true; }

  //! @copydoc SignalLoggerBase::publishData
  virtual bool publishData() override { return true; }

  //! @copydoc SignalLoggerBase::saveLoggerData
  virtual bool saveLoggerData([[maybe_unused]] const signal_logger::LogFileTypeSet & logfileTypes) override { return true; }

  //! @copydoc SignalLoggerBase::stopAndSaveLoggerData
  virtual bool stopAndSaveLoggerData([[maybe_unused]] const signal_logger::LogFileTypeSet & logfileTypes) override { return true; }

 protected:
  //! @copydoc SignalLoggerBase::workerSaveData
  virtual bool workerSaveData([[maybe_unused]] const std::string & logFileName, [[maybe_unused]] const std::string & pathWithPrefix, [[maybe_unused]] const signal_logger::LogFileTypeSet & logfileTypes) override { return false; };

};

}
