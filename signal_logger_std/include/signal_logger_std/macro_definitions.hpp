/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#define ADD_VAR_IMPLEMENTATION(TYPE, NAME, ...) \
    virtual void add##NAME(const TYPE & var, \
                           const std::string& name, \
                           const std::string& group, \
                           const std::string& unit, \
                           bool update) { \
      log_elements_[name] = new LogElementStd<TYPE>(const_cast<TYPE*>(&var), signal_logger::LOGGER_PREFIX + "/" + group + "/" + name, unit, buffer_size_, &file_); \
    } /*
 */
