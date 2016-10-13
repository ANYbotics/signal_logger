/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/macro_definitions.hpp"
#include "signal_logger/SignalLoggerBase.hpp"

// signal logger std
#include "signal_logger_std/LogElementStd.hpp"

/**
 *  Macro that implements the add function for all types that are currently supported
 *  @param TYPE data type of the log variable
 *  @param NAME name of the data type added to add in function name
 */
#define ADD_STD_VAR_IMPLEMENTATION(TYPE, NAME, ...) \
    /** Function definition to add variable of type TYPE to the logger. */ \
    /** @param var            log variable */ \
    /** @param name           name of the log variable*/ \
    /** @param group          logger group the variable belongs to*/ \
    /** @param unit           unit of the log variable*/ \
    /** @param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)*/ \
    /** @param action         log action of the log variable*/ \
    /** @param bufferSize     size of the buffer storing log elements*/ \
    /** @param bufferType     determines type of the buffer*/ \
    virtual void add##NAME(const TYPE & var, \
                           const std::string & name, \
                           const std::string & group, \
                           const std::string & unit, \
                           const std::size_t divider, \
                           const signal_logger::LogElementAction action, \
                           const std::size_t bufferSize, \
                           const signal_logger::BufferType bufferType) \
    { \
      std::string elementName = std::string{signal_logger::SignalLoggerBase::LOGGER_DEFAULT_PREFIX} + "/" + group + "/" + name; \
      logElements_[elementName].reset(new signal_logger_std::LogElementStd<TYPE>(const_cast<TYPE*>(&var), elementName , \
              unit, divider, action, bufferSize, bufferType, &headerStream_, &dataStream_)); \
    } /*
                            */
