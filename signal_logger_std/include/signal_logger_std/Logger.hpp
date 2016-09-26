/*
 * Logger.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <signal_logger_std/LogElementStd.hpp>
#include "signal_logger/LogElementInterface.hpp"

#include <unordered_map>

#include <ros/node_handle.h>

namespace signal_logger_std {

const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const bool LOGGER_DEFAULT_UPDATE            = false;
const std::string LOGGER_DEFAULT_FILENAME   = "logger.script";
const std::string LOGGER_PREFIX = "/log";

class Logger
{
 public:
  Logger(std::size_t buffer_size): buffer_size_(buffer_size), file_() {}
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
    log_elements_[name] = new LogElementStd<ValueType_>(var, LOGGER_PREFIX + "/" + group + "/" + name, unit, buffer_size_, &file_);

  }

  void collectData()
  {
    for(auto & elem : log_elements_) elem.second->collectData();
  }

  void publishData()
  {
    std::string filename = "mydata3.txt";
    file_.open(filename, std::ios::out | std::ios::app);
    file_ << "Begin file";
    for(auto & elem : log_elements_) {
      elem.second->writeHeaderToLogFile();
    }
    file_.close();
    file_.open(filename, std::ios::out | std::ios::app | std::ios::binary);
    for(auto & elem : log_elements_) {
      std::cout << " Publish "<<elem.second->getName()<<std::endl;
      elem.second->writeDataToLogFile();
    }

    file_.close();
  }
 private:
  std::size_t buffer_size_;
  std::ofstream file_;
  std::unordered_map<std::string, signal_logger::LogElementInterface *> log_elements_;
};

} /* namespace signal_logger */
