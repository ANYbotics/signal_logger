/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

//! This macro generates the implementation of the add function for all types that are currently supported
#define ADD_VAR_IMPLEMENTATION(TYPE, NAME, ...) \
    virtual void add##NAME(const TYPE & var, \
                           const std::string& name, \
                           const std::string& group, \
                           const std::string& unit, \
                           bool update) { \
      std::string elementName = signal_logger::LOGGER_PREFIX + "/" + group + "/" + name; \
      logElements_[elementName] = new LogElementStd<TYPE>(const_cast<TYPE*>(&var), elementName , unit, bufferSize_, &file_); \
    } /*
 */
