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
  LogElementRos(ValueType_ * ptr,
                const std::string & name,
                const std::string & unit,
                const std::size_t divider,
                const signal_logger::LogElementAction action,
                const std::size_t bufferSize,
                const signal_logger::BufferType bufferType,
                std::stringstream * headerStream,
                std::stringstream * dataStream,
                ros::NodeHandle * nh) :
      signal_logger_std::LogElementStd<ValueType_>(ptr, name, unit, divider, action, bufferSize, bufferType, headerStream, dataStream),
      nh_(nh),
      pub_(),
      publishCount_(0)
  {
    msg_.reset(new MsgType());
  }

  //! Destructor
  virtual ~LogElementRos()
  {

  }

  //! Reads buffer and publishes data via ros
  void publishData(const signal_logger::LogElementBase<signal_logger::TimestampPair> & time) override
  {
    ValueType_ data;

    if(this->buffer_.read(&data))
    {
      if( (publishCount_*this->getDivider()) < time.getBufferSize()) {
        signal_logger::TimestampPair tsp_now = time.getNthTimestep(publishCount_*this->getDivider());
        ros::Time now = ros::Time(tsp_now.first, tsp_now.second);
        traits::slr_traits<ValueType_>::updateMsg(&data, msg_, now);
        pub_.publish(msg_);
        ++publishCount_;
      }
      else {
        MELO_ERROR_STREAM("Buffer of time in Logger is too small.");
      }
    }
  }

  void restartElement() override
  {
    signal_logger_std::LogElementStd<ValueType_>::restartElement();
    publishCount_ = std::size_t(0);
  }

  void initializeElement() override
  {
    signal_logger_std::LogElementStd<ValueType_>::initializeElement();
    pub_ = nh_->advertise<MsgType>(this->getName(), 1);
  }

  void shutdownElement() override
  {
    pub_.shutdown();
    signal_logger_std::LogElementStd<ValueType_>::shutdownElement();
  }

 protected:
  //! ros nodehandle
  ros::NodeHandle * nh_;
  //! ros publisher
  ros::Publisher pub_;
  //! message pointer
  MsgTypePtr msg_;
  //! no published items
  std::size_t publishCount_;

};

} /* namespace signal_logger */

