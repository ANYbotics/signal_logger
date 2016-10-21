/**
* @file 	  signal_logger.hpp
* @author 	  Christian Gehring, C. Dario Bellicoso, Gabriel Hottiger
* @date		  June 26, 2013
*/

#pragma once

#ifdef E
#undef E
#endif

#include "signal_logger/SignalLoggerBase.hpp"
#include <memory>
#include "assert.h"

namespace signal_logger {

//! Reference to the logger
extern std::shared_ptr<SignalLoggerBase> logger;

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
void add( const ValueType_ & var,
          const std::string & name,
          const std::string & group       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
          const std::string & unit        = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
          const std::size_t divider       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
          const LogElementAction action   = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
          const std::size_t bufferSize    = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
          const BufferType bufferType     = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE);

/** Function implementation to add eigen matrices as their underlying type to the logger.
  *@param var            pointer to log matrix
  *@param names          name of every entry of the matrix
  *@param group          logger group the variable belongs to
  *@param unit           unit of the log variable
  *@param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)
  *@param action         log action of the log variable
  *@param bufferSize     size of the buffer storing log elements
  *@param bufferType     determines type of buffer
  */
template<typename ValueType_>
typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type
add(const ValueType_ & var,
    const signal_logger::MatrixXstring & names,
    const std::string & group       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
    const std::string & unit        = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
    const std::size_t divider       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
    const LogElementAction action   = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
    const std::size_t bufferSize    = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_SIZE,
    const BufferType bufferType     = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE)
{
  assert(names.rows() == var.rows() && "rows() have different size in add");
  assert(names.cols() == var.cols() && "cols() have different size in add");

  for(std::size_t i = 0; i < var.size(); ++i)
  {
    add<typename ValueType_::Scalar>(static_cast<const typename ValueType_::Scalar * const>(var.data() + i),
                                     static_cast<std::string>(*(names.data() + i)),
                                     group,
                                     unit,
                                     divider,
                                     action,
                                     bufferSize,
                                     bufferType);
  }
}


} // end namespace
