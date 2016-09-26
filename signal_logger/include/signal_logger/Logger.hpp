/*
 * Logger.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <signal_logger/LogElementBase.hpp>
#include "signal_logger/LogElementInterface.hpp"

#include <unordered_map>

namespace signal_logger {

const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const bool LOGGER_DEFAULT_UPDATE            = false;
const std::string LOGGER_DEFAULT_FILENAME   = "logger.script";
const std::string LOGGER_PREFIX = "/log";

class Logger
{
 public:
  Logger(std::size_t buffer_size): buffer_size_(buffer_size) {}
  virtual ~Logger() {}

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
    log_elements_[name] = new LogElementBase<ValueType_>(var, LOGGER_PREFIX + "/" + group + "/" + name, unit, buffer_size_);
  }

  void collectData()
  {
    for(auto & elem : log_elements_) elem.second->collectData();
  }

  void publishData()
  {
    for(auto & elem : log_elements_) {
      std::cout << " Publish "<<elem.second->getName()<<std::endl;
      elem.second->publishData();
    }
  }
 private:
  std::size_t buffer_size_;
  std::unordered_map<std::string, LogElementInterface *> log_elements_;
};

} /* namespace signal_logger */
