/*
 * LogElementRos.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/LogElementBase.hpp"
#include "signal_logger_ros/signal_logger_ros_traits.hpp"

// ros
#include <ros/publisher.h>
#include <ros/node_handle.h>

namespace signal_logger_ros {

//! Log element for ros logging
template <typename ValueType_>
class LogElementRos: public signal_logger::LogElementBase<ValueType_>
{
  //! convinience typedefs
  using MsgType = typename traits::slr_traits<ValueType_>::msgtype;
  using MsgTypePtr = typename traits::slr_traits<ValueType_>::msgtypePtr;

 public:
  /** Constructor
   * @param ptr     pointer to the data that shall be logged
   * @param name    name of the log element
   * @param unit    unit of the logged data
   * @param buffer_size size of the buffer (old elements will always be overwritten)
   * @param nh      nodehandle to advertise the publisher
   * @return log element
  */
  LogElementRos(ValueType_ * ptr,
                const std::string & name,
                const std::string & unit,
                const std::size_t buffer_size,
                const ros::NodeHandle & nh) :
    signal_logger::LogElementBase<ValueType_>(ptr, name, unit, buffer_size),
    nh_(nh)
  {
    //! A buffer is already provided, publisher should not create internal one
    pub_ = nh_.advertise<MsgType>(name, 1);
    msg_.reset(new MsgType());
  }

  //! Destructor
  virtual ~LogElementRos()
  {

  }

  //! Reads buffer and publishes data via ros
  void publishData()
  {
    ValueType_ * ptr = new ValueType_();
    signal_logger::LogElementBase<ValueType_>::readDataFromBuffer(ptr);
    ros::Time now = ros::Time::now();
    traits::slr_traits<ValueType_>::updateMsg(ptr, msg_, now);
    pub_.publish(msg_);
  }

  //! These functions do nothing for a ros element
  void writeHeaderToLogFile() { }
  void writeDataToLogFile() { }

 protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  MsgTypePtr msg_;


};

} /* namespace signal_logger */

