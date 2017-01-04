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

// rosbag
#include <rosbag/bag.h>

namespace signal_logger_ros {

//! Log element for ros logging
template <typename ValueType_>
class LogElementRos: public signal_logger_std::LogElementStd<ValueType_>
{
  //! convinience typedefs
  using MsgType = typename traits::slr_msg_traits<ValueType_>::msgtype;
  using MsgTypePtr = typename traits::slr_msg_traits<ValueType_>::msgtypePtr;

 public:
  /** Constructor
   *  @param ptr          pointer to the log var
   *  @param name         name of the log var
   *  @param unit         unit of the log var
   *  @param divider      log_freq = ctrl_freq/divider
   *  @param action       save, publish or save and publish
   *  @param bufferSize   size of the buffer (bufferSize elements of type ValueType_)
   *  @param bufferType   type of the buffer
   *  @param headerStream pointer to the header stream of the binary log file
   *  @param dataStream   pointer to the data stream of the binary log file
   *  @param nh           ros nodehandle for the ros publisher
   *  @param bagWriter    reference to the bagfile writer object
   *  @param saveToBag    flag if elemt shall be save to bag
   */
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
                const std::shared_ptr<rosbag::Bag> & bagWriter) :
                  signal_logger_std::LogElementStd<ValueType_>(ptr, name, unit, divider, action, bufferSize, bufferType, headerStream, dataStream),
                  nh_(nh),
                  bagWriter_(bagWriter),
                  pub_()
  {
    msg_.reset(new MsgType());
    msgSave_.reset(new MsgType());
  }

  //! Destructor
  virtual ~LogElementRos()
  {

  }

  //! Save Data to file
  void saveDataToLogFile(const std::vector<signal_logger::TimestampPair> & times,
                         unsigned int nrCollectDataCalls,
                         signal_logger::LogFileType type = signal_logger::LogFileType::BINARY) override
  {
    // If binary log file -> call super class
    if(type == signal_logger::LogFileType::BINARY) {
      signal_logger_std::LogElementStd<ValueType_>::saveDataToLogFile(times, nrCollectDataCalls, type);
      return;
    }
    else if(type == signal_logger::LogFileType::BAG) {
      // Lock the copy mutex
      std::unique_lock<std::mutex> lock(this->copyMutex_);

      // Oldest entry in the time buffer
      std::size_t startIdx = 0;

      if(this->isBufferLoopingCopy_) {
        /* Last index of time: (times.size() - 1)
         * Index of newest time corresponding to a data point:  (nrCollectDataCalls - 1) % this->dividerCopy_
         * Offset of oldest time that corresponds to a data point: (this->bufferCopy_.size()-1) * this->dividerCopy_
         */
        startIdx = (times.size() - 1) - (nrCollectDataCalls - 1) % this->dividerCopy_ - (this->bufferCopy_.size()-1) * this->dividerCopy_;
      }
      
      for(std::size_t i = 0; i < this->bufferCopy_.size(); ++i) {
        // Get time at data point
        signal_logger::TimestampPair tsp_now = times.at(startIdx + i*this->dividerCopy_);
        ros::Time now = ros::Time(tsp_now.first, tsp_now.second);
        // Update msg
        ValueType_ data = this->bufferCopy_.at(i);
        traits::slr_update_traits<ValueType_>::updateMsg(&data, msgSave_, now);
        // Write to bag
        bagWriter_->write(this->nameCopy_, now, *msgSave_);
      }
    }
  }

  //! Reads buffer and publishes data via ros
  void publishData(const signal_logger::LogElementBase<signal_logger::TimestampPair> & time, unsigned int nrCollectDataCalls) override
  {
    if(this->noUnreadItemsInBuffer())
    {
      // Local vars
      signal_logger::TimestampPair tsp_now;
      ros::Time now;
      ValueType_ data;

      {
        std::unique_lock<std::mutex> lock(this->mutex_);

        {
          std::unique_lock<std::mutex> timeLock(time.acquireMutex());

          // Define time index depending on buffer type
          std::size_t idx = (time.noItemsInBuffer() - 1) - (this->noItemsInBuffer()-this->noUnreadItemsInBuffer())*this->divider_;

          if(this->buffer_.getType() == signal_logger::BufferType::LOOPING) {
            idx = (time.noItemsInBuffer() - 1) - (nrCollectDataCalls - 1) % this->divider_
                - (this->noItemsInBuffer()-this->noUnreadItemsInBuffer())*this->divider_;
          }

          // get time stamp
          signal_logger::TimestampPair tsp_now = time.getTimeStampAtPosition(idx);

        } // unlock time mutex

        // convert to ros time
        ros::Time now = ros::Time(tsp_now.first, tsp_now.second);

        // Read from buffer and transform to message via trait
        this->buffer_.read(&data);

      } // unlock elements mutex

      // publish over ros
      traits::slr_update_traits<ValueType_>::updateMsg(&data, msg_, now);
      {
        std::unique_lock<std::mutex> lock(this->publishMutex_);
        pub_.publish(msg_);
      }

    }
  }

  //! Update the element, shutdown/advertise the ros publisher
  void updateElement() override {
    std::unique_lock<std::mutex> lock(this->publishMutex_);
    if(this->isPublished() && this->isEnabled()) {
      pub_ = nh_->advertise<MsgType>(this->getName(), 1);
    }  else {
      pub_.shutdown();
    }
  }

  //! Cleanup the element (shutdown ros publisher)
  void cleanupElement() override {
    signal_logger_std::LogElementStd<ValueType_>::cleanupElement();
    std::unique_lock<std::mutex> lock(this->publishMutex_);
    pub_.shutdown();
  }

 protected:
  //! ros nodehandle
  ros::NodeHandle * nh_;
  //! bag writer
  const std::shared_ptr<rosbag::Bag> & bagWriter_;
  //! ros publisher
  ros::Publisher pub_;
  //! publisher mutex
  std::mutex publishMutex_;
  //! message pointer
  MsgTypePtr msg_;
  MsgTypePtr msgSave_;

};

} /* namespace signal_logger */
