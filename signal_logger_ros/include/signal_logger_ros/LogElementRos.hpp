/*
 * LogElementRos.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger_std/LogElementStd.hpp"
#include "signal_logger_ros/signal_logger_ros_traits.hpp"

// ros
#include <ros/publisher.h>
#include <ros/node_handle.h>

namespace signal_logger_ros {

//! Log element for ros logging
template <typename ValueType_>
class LogElementRos: public signal_logger_std::LogElementStd<ValueType_>
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
                const std::size_t divider,
                const signal_logger::LogElementInterface::LogElementAction action,
                const std::size_t bufferSize,
                const bool isBufferLooping,
                std::stringstream * headerStream,
                std::stringstream * dataStream,
                const ros::NodeHandle & nh) :
    signal_logger_std::LogElementStd<ValueType_>(ptr, name, unit, divider, action, bufferSize, isBufferLooping, headerStream, dataStream),
    nh_(nh),
    publishCount_(0)
  {
    //! A buffer is already provided, publisher should not create internal one
    msg_.reset(new MsgType());
  }

  //! Destructor
  virtual ~LogElementRos()
  {

  }

  //! Reads buffer and publishes data via ros
  void publishData(signal_logger::LogElementBase<signal_logger::TimestampPair> * time)
  {
    ValueType_ data;
    if(this->buffer_.read(&data))
    {
      signal_logger::TimestampPair tsp_now = time->copyElementFromBack(publishCount_*this->getDivider());
      ros::Time now = ros::Time(tsp_now.first, tsp_now.second);
      traits::slr_traits<ValueType_>::updateMsg(&data, msg_, now);
      pub_.publish(msg_);
    }
    ++publishCount_;
  }

  void initializeElement()
  {
    signal_logger_std::LogElementStd<ValueType_>::initializeElement();
    pub_ = nh_.advertise<MsgType>(this->getName(), 1);
  }
  void shutdownElement()
  {
    pub_.shutdown();
    signal_logger_std::LogElementStd<ValueType_>::shutdownElement();
  }

 protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  MsgTypePtr msg_;
  std::size_t publishCount_;

};

} /* namespace signal_logger */

