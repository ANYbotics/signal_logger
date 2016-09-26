/*
 * SignalLoggerBase.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "signal_logger/SignalLoggerInterface.hpp"

namespace signal_logger {

class SignalLoggerBase: SignalLoggerInterface {

  template<typename ValueType_>
  void addVariableToLog(ValueType_ * var,
                        const std::string& name,
                        const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                        const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                        bool update = LOGGER_DEFAULT_UPDATE)
  {



    if(log_elements_.find(name) != log_elements_.end()) {
      printf("A signal with the same name %s is already logged. Overwrite.", name.c_str());
    }
    LogElementInterface * element = this->createElement(LOGGER_PREFIX + "/" + group + "/" + name, unit, buffer_size_);
    static_cast<LogElementBase<ValueType_>>
//        new LogElementBase<ValueType_>(, unit, buffer_size_);
//
//    log_elements_[name] =
  }

 private:
  std::size_t buffer_size_;
  std::unordered_map<std::string, signal_logger::LogElementInterface *> log_elements_;


};



}

