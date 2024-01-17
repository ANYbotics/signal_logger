/*!
 * @file	    typedefs.hpp
 * @author	    Gabriel Hottiger
 * @date	    Jan 30, 2017
 */

#pragma once


// signal logger core
#include "signal_logger_core/signal_logger_traits.hpp"

// STL
#include <unordered_set>

namespace signal_logger {

//! Forward declaration of LogElementBase
template <typename ValueType_>
class LogElementBase;

using TimeElement = LogElementBase<TimestampPair>;


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
  CSV = 1, /*!< 1 */
  BAG = 2, /*!< 2 */
};

//! Helper for subclass add functions
struct both_slashes {
  bool operator()(char a, char b) const {
    return a == '/' && b == '/';
  }
};

//! Vector type
template <typename T>
using vector_type = typename std::conditional<traits::is_eigen_matrix<T>::value,
  std::vector<T, Eigen::aligned_allocator<T>>, std::vector<T>>::type;


//! Enum class hash
struct EnumClassHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

using LogFileTypeSet = std::unordered_set<LogFileType, EnumClassHash>;

}
