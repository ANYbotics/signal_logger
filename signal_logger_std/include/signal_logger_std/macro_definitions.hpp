/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/macro_definitions.hpp"
#include "signal_logger_std/LogElementStd.hpp"
#include "signal_logger/SignalLoggerBase.hpp"

//! This macro generates the implementation of the add function for all types that are currently supported
#define ADD_STD_VAR_IMPLEMENTATION(TYPE, NAME, ...) \
    virtual void add##NAME(const TYPE & var, \
                           const std::string& name, \
                           const std::string& group, \
                           const std::string& unit, \
                           const std::size_t divider, \
                           const signal_logger::LogElementInterface::LogElementAction & action, \
                           const std::size_t bufferSize, \
                           const bool bufferLooping) \
    { \
      std::string elementName = signal_logger::LOGGER_PREFIX + "/" + group + "/" + name; \
      logElements_[elementName].reset(new signal_logger_std::LogElementStd<TYPE>(const_cast<TYPE*>(&var), elementName , \
              unit, divider, action, bufferSize, bufferLooping, &headerStream_, &dataStream_)); \
    } /*
                            */
