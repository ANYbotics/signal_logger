/*
 * BufferInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: gabrielhottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementBase.hpp"

// Eigen
#include "Eigen/Core"

// ros
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include "signal_logger_ros/signal_logger_ros_traits.hpp"

namespace signal_logger_ros {

template <typename ValueType_, typename Enable_ = void>
class LogElementRos: public signal_logger::LogElementBase<ValueType_>
{
  using MsgType = typename traits::slr_traits<ValueType_>::msgtype;
  using MsgTypePtr = typename traits::slr_traits<ValueType_>::msgtypePtr;

 public:
  LogElementRos(ValueType_ * ptr, std::string name, std::size_t buffer_size, const ros::NodeHandle & nh) :
    signal_logger::LogElementBase<ValueType_>(ptr, name, buffer_size),
    nh_(nh)
  {
    pub_ = nh_.advertise<MsgType>(name, 1);
    msg_.reset(new MsgType());
  }

  virtual ~LogElementRos() {

  }

  void publish() {
    ValueType_ * ptr = new ValueType_();
    signal_logger::LogElementBase<ValueType_>::readBuffer(ptr);
    ros::Time now = ros::Time::now();
    traits::slr_traits<ValueType_>::updateMsg(ptr, msg_, now);
    pub_.publish(msg_);
  }

 protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  MsgTypePtr msg_;


};

} /* namespace signal_logger */

