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

// bageditor
#include <bageditor/BagWriter.hpp>

namespace signal_logger_ros {

//! Log element for ros logging
template <typename ValueType_>
class LogElementRos: public signal_logger_std::LogElementStd<ValueType_>
{
  //! convinience typedefs
  using MsgType = typename traits::slr_msg_traits<ValueType_>::msgtype;
  using MsgTypePtr = typename traits::slr_msg_traits<ValueType_>::msgtypePtr;

 public:
  LogElementRos(const ValueType_ * const ptr,
                const std::string & name,
                const std::string & unit,
                const std::size_t divider,
                const signal_logger::LogElementAction action,
                const std::size_t bufferSize,
                const signal_logger::BufferType bufferType,
                std::stringstream * headerStream,
                std::stringstream * dataStream,
                ros::NodeHandle * nh,
                const std::shared_ptr<bageditor::BagWriter> & bagWriter) :
                  signal_logger_std::LogElementStd<ValueType_>(ptr, name, unit, divider, action, bufferSize, bufferType, headerStream, dataStream),
                  nh_(nh),
                  bagWriter_(bagWriter),
                  pub_()
  {
    msg_.reset(new MsgType());
  }

  //! Destructor
  virtual ~LogElementRos()
  {

  }

  //! Save Data to file
  void saveDataToLogFile(const signal_logger::LogElementBase<signal_logger::TimestampPair> & time, unsigned int nrCollectDataCalls) override
  {
    std::unique_lock<std::mutex> lock(this->mutex_);

    // Copy data and time buffers
    std::vector<ValueType_> values = this->buffer_.copyBuffer();

    // Define start index
    std::size_t lastEntryIdx = (time.noItemsInBuffer() - 1) - (nrCollectDataCalls - 1) % this->divider_;

    // Write to bag
    for(std::size_t i = 0; i < values.size(); ++i) {
      signal_logger::TimestampPair tsp_now = time.getTimeStampAtPosition(lastEntryIdx - i*this->divider_);
      ros::Time now = ros::Time(tsp_now.first, tsp_now.second);
      ValueType_ data = values.at(values.size() - i - 1);
      traits::slr_update_traits<ValueType_>::updateMsg(&data, msg_, now);
      bagWriter_->writeStampedMessageToTopic(this->getName(), *msg_);
    }

  }

  //! Reads buffer and publishes data via ros
  void publishData(const signal_logger::LogElementBase<signal_logger::TimestampPair> & time, unsigned int nrCollectDataCalls) override
  {
    if(this->noUnreadItemsInBuffer())
    {
      std::unique_lock<std::mutex> lock(this->mutex_);

      std::size_t idx = (time.noItemsInBuffer() - 1) - (nrCollectDataCalls - 1) % this->divider_
                        - (this->noItemsInBuffer()-this->noUnreadItemsInBuffer())*this->divider_;
      signal_logger::TimestampPair tsp_now = time.getTimeStampAtPosition(idx);

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

  void updateElement() override {
    if(this->isPublished() && this->isEnabled()) {
      pub_ = nh_->advertise<MsgType>(this->getName(), 1);
    }  else {
      pub_.shutdown();
    }
  }

  void cleanupElement() override {
    signal_logger_std::LogElementStd<ValueType_>::cleanupElement();
    pub_.shutdown();
  }


 protected:
  //! ros nodehandle
  ros::NodeHandle * nh_;
  //! ros nodehandle
  const std::shared_ptr<bageditor::BagWriter> & bagWriter_;
  //! ros publisher
  ros::Publisher pub_;
  //! message pointer
  MsgTypePtr msg_;
};

} /* namespace signal_logger */
