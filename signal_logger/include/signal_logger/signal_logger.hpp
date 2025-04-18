/*!
 * @file     signal_logger.hpp
 * @author   Christian Gehring, C. Dario Bellicoso, Gabriel Hottiger
 * @date     June 23, 2013
 * @brief    Providing add function depending on current logger type.
 */


#pragma once

#ifdef E
#undef E
#endif

#include "signal_logger_core/typedefs.hpp"
#include "signal_logger_core/SignalLoggerBase.hpp"
#include "signal_logger_ros/SignalLoggerRos.hpp"
#include "signal_logger_std/SignalLoggerStd.hpp"
#include "signal_logger/SignalLoggerNone.hpp"

#include <memory>
#include <optional>
#include "assert.h"

namespace signal_logger {

//! Reference to the logger
extern std::shared_ptr<SignalLoggerBase> logger;

//! Get the logger type at runtime
enum class LoggerType: int {
  TypeUnknown = -1,/*!< -1 */
  TypeNone    = 0,/*!< 0 */
  TypeStd     = 1,/*!< 1 */
  TypeRos     = 2/*!< 2 */
};

//! @return the logger type
LoggerType getLoggerType();

void setSignalLoggerNone();

void setSignalLoggerStd();

#ifndef ROS2_BUILD
void setSignalLoggerRos(ros::NodeHandle* nh);
#else /* ROS2_BUILD */
void setSignalLoggerRos(rclcpp::Node::SharedPtr nh);
#endif /* ROS2_BUILD */

#ifndef ROS2_BUILD
void setSignalLogger(const std::string& name, ros::NodeHandle* nh);
#else /* ROS2_BUILD */
void setSignalLogger(const std::string& name, rclcpp::Node::SharedPtr nh);
#endif /* ROS2_BUILD */


/** Add variable to logger. This is a default implementation if no specialization is provided an error is posted.
  * @tparam ValueType_       Data type of the logger element
  * @param  var              Pointer to log variable
  * @param  name             name of the log variable
  * @param  group            logger group the variable belongs to
  * @param  unit             unit of the log variable
  * @param  divider          divider is defining the update frequency of the logger element (ctrl_freq/divider)
  * @param  action           log action of the log variable
  * @param  bufferType       determines the buffer type
  * @param  bufferSize       optional size of the buffer storing log elements
  */
template<typename ValueType_>
void add( const ValueType_ & var,
          const std::string & name,
          const std::string & group                      = SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
          const std::string & unit                       = SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
          const std::size_t divider                      = SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
          const LogElementAction action                  = SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
          const BufferType bufferType                    = SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE,
          const std::optional<std::size_t> bufferSize    = std::nullopt)
{
      signal_logger_ros::SignalLoggerRos* slRos = dynamic_cast<signal_logger_ros::SignalLoggerRos*>(logger.get());
      if(slRos) {
        slRos->add<ValueType_>(&var, name, group, unit, divider, action, bufferType, bufferSize);
        return;
      }

    signal_logger_std::SignalLoggerStd* slStd = dynamic_cast<signal_logger_std::SignalLoggerStd*>(logger.get());
    if(slStd) {
      slStd->add<ValueType_>(&var, name, group, unit, divider, action, bufferType, bufferSize);
      return;
    }

    SignalLoggerNone* slNone = dynamic_cast<SignalLoggerNone*>(logger.get());
    if(slNone) {
      slNone->add<ValueType_>(&var, name, group, unit, divider, action, bufferType, bufferSize);
      return;
    }
}

/** Function implementation to add eigen matrices as their underlying type to the logger.
  *@param var            pointer to log matrix
  *@param names          name of every entry of the matrix
  *@param group          logger group the variable belongs to
  *@param unit           unit of the log variable
  *@param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)
  *@param action         log action of the log variable
  *@param bufferType     determines type of buffer
  *@param bufferSize     optional size of the buffer storing log elements
  */
template<typename ValueType_>
typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type
add(const ValueType_ & var,
    Eigen::Ref<MatrixXstring> names,
    const std::string & group                      = SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME,
    const std::string & unit                       = SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT,
    const std::size_t divider                      = SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER,
    const LogElementAction action                  = SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION,
    const BufferType bufferType                    = SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE,
    const std::optional<std::size_t> bufferSize    = std::nullopt)
{
  assert(names.rows() == var.rows() && "rows() have different size in add");
  assert(names.cols() == var.cols() && "cols() have different size in add");

  for(std::size_t i = 0; i < var.size(); ++i)
  {
    add<typename ValueType_::Scalar>(*static_cast<const typename ValueType_::Scalar * const>(var.data() + i),
                                     static_cast<std::string>(*(names.data() + i)),
                                     group,
                                     unit,
                                     divider,
                                     action,
                                     bufferType,
                                     bufferSize);
  }
}


} // end namespace
