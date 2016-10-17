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
  using MsgType = typename traits::slr_msg_traits<ValueType_>::msgtype;
  using MsgTypePtr = typename traits::slr_msg_traits<ValueType_>::msgtypePtr;

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

    if(this->noUnreadItemsInBuffer())
    {
      std::unique_lock<std::mutex> lock(this->mutex_);
      signal_logger::TimestampPair tsp_now;
      if(this->getBufferType() == signal_logger::BufferType::FIXED_SIZE)
      {
        /* Read from back of time buffer
         * (no_items_data - no_unread_items_data)*div -> shift from the first time buffer entry
         * (no_items_time-1) -> index of the first time
         * Index in time buffer , idx = (no_items_time-1) - (no_items_data - no_unread_items_data)*div;
        */
        tsp_now = time.getTimeStampAtPosition( (time.noItemsInBuffer()-1) - (this->noItemsInBuffer()-this->noUnreadItemsInBuffer())*this->divider_);
      }
      else
      {
        /* Read from back of time buffer
         * no_items_time % div_data -> latest element of timebuffer that corresponds to a value
         * (no_unread_items_data - 1 )*div -> shift from the latest corresponding entry
         * Index in time buffer , idx = (no_items_time % div_data) + (no_unread_items_data - 1 )*div
        */
        // If time is not synchronized (time is 1 collection ahead), correct for this
        std::size_t offset = this->isTimeSynchronzied()?time.noItemsInBuffer()%this->divider_:((time.noItemsInBuffer()-1)%this->divider_)+1;
        tsp_now = time.getTimeStampAtPosition( offset + (this->noUnreadItemsInBuffer() - 1)*this->divider_);
      }
        // convert to ros time
        ros::Time now = ros::Time(tsp_now.first, tsp_now.second);

        // Read from buffer and transform to message via trait
        ValueType_ data;
        this->buffer_.read(&data);

        // Unlock for publishing
        lock.unlock();

        // publish over ros
        traits::slr_update_traits<ValueType_>::updateMsg(&data, msg_, now);
        pub_.publish(msg_);
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

