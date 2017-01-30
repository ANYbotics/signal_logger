/*!
 * @file	  typedefs.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan 30, 2017
 */

#pragma once

#include <cstddef>

namespace signal_logger {

//! Enum containing possible buffer types
enum class BufferType: unsigned int
{
  FIXED_SIZE = 0,/*!< 0 */
  LOOPING = 1,/*!< 1 */
  EXPONENTIALLY_GROWING = 2/*!< 2 */
};

//! Enum containing possible logging actions
enum class LogElementAction: unsigned int
{
  SAVE_AND_PUBLISH = 0,/*!< 0 */
  SAVE = 1,/*!< 1 */
  PUBLISH = 2/*!< 2 */
};

//! Enum containing possible log file types
enum class LogFileType: unsigned int
{
  BINARY = 0,/*!< 0 */
  BAG = 1, /*!< 1 */
  BINARY_AND_BAG = 2/*!< 2 */
};

//! Default values
static constexpr const char* LOG_ELEMENT_DEFAULT_GROUP_NAME   = "/log/";
static constexpr const char* LOG_ELEMENT_DEFAULT_UNIT         = "-";
static constexpr std::size_t LOG_ELEMENT_DEFAULT_DIVIDER      = 1;
static constexpr LogElementAction LOG_ELEMENT_DEFAULT_ACTION  = LogElementAction::SAVE_AND_PUBLISH;
static constexpr std::size_t LOG_ELEMENT_DEFAULT_BUFFER_SIZE  = 1000;
static constexpr BufferType LOG_ELEMENT_DEFAULT_BUFFER_TYPE   = BufferType::LOOPING;

//! Signal logger default options
static constexpr const double LOGGER_DEFAULT_MAXIMUM_LOG_TIME     = 0.0;
static constexpr const char*  LOGGER_DEFAULT_SCRIPT_FILENAME      = "logger.yaml";
static constexpr const char*  LOGGER_DEFAULT_PREFIX               = "/log";
static constexpr const double LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME = 10.0;

//! Helper for subclass add functions
struct both_slashes {
  bool operator()(char a, char b) const {
    return a == '/' && b == '/';
  }
};

}
