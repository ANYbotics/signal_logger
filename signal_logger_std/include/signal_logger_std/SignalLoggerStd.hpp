/*!
 * @file     SignalLoggerStd.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 23, 2016
 * @brief    Implementation of a Std signal logger. Provides binary file storing functionality.
 */

#pragma once

// signal logger
#include "signal_logger_core/SignalLoggerBase.hpp"
#include "signal_logger_std/LogElementStd.hpp"

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

  /** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
   * @tparam ValueType_       Data type of the logger element
   * @param  var              Pointer to log variable
   * @param  name             name of the log variable
   * @param  group            logger group the variable belongs to
   * @param  unit             unit of the log variable
   * @param  divider          divider is defining the update frequency of the logger element (ctrl_freq/divider)
   * @param  action           log action of the log variable
   * @param  bufferSize       size of the buffer storing log elements
   * @param  bufferType       determines the buffer type
   */
  template<typename ValueType_>
  void add( const ValueType_ * const var,
            const std::string & name,
            const std::string & group                      = LOG_ELEMENT_DEFAULT_GROUP_NAME,
            const std::string & unit                       = LOG_ELEMENT_DEFAULT_UNIT,
            const std::size_t divider                      = LOG_ELEMENT_DEFAULT_DIVIDER,
            const signal_logger::LogElementAction action   = LOG_ELEMENT_DEFAULT_ACTION,
            const std::size_t bufferSize                   = LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
            const signal_logger::BufferType bufferType     = LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
  {
    std::string elementName = std::string{signal_logger::SignalLoggerBase::LOGGER_DEFAULT_PREFIX} + "/" + group + "/" + name;
    elementName.erase(std::unique(elementName.begin(), elementName.end(), signal_logger::both_slashes()), elementName.end());
    {
      boost::unique_lock<boost::shared_mutex> lock(newElementsMapMutex_);
      logElementsToAdd_[elementName].reset(new signal_logger_std::LogElementStd<ValueType_>(var, elementName , unit, divider, action,
                                                                                            bufferSize, bufferType, &headerStream_, &dataStream_));
    }

  }

  //! Save all the buffered data into a log file
  virtual bool workerSaveData(const std::string & logFileName, signal_logger::LogFileType logfileType);

  //! @return the logger type
  virtual LoggerType getLoggerType() const { return SignalLoggerBase::LoggerType::TypeStd; }

  /** Resets the pointer to the logelement
   * @param buffertype type of the time buffer
   * @param maxLogTime maximal time logging
   */
  virtual bool resetTimeLogElement(signal_logger::BufferType buffertype,
                                   double maxLogTime = signal_logger::SignalLoggerBase::LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME) override;

 protected:
  //! Binary log file
  std::ofstream file_;
  //! Header stream
  std::stringstream headerStream_;
  //! Data stream
  std::stringstream dataStream_;
};

} /* namespace signal_logger_std */
