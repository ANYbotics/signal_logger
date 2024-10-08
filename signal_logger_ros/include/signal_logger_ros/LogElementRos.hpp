/*!
 * @file     LogElementRos.hpp
 * @author   Gabriel Hottiger
 * @date     Sep 22, 2016
 * @brief    Implementation of a Log element for ros logging.
 */

#pragma once

// signal logger
#include "signal_logger_std/LogElementStd.hpp"
#include "signal_logger_ros/signal_logger_ros_traits.hpp"

// ros
#ifndef ROS2_BUILD
#include <ros/publisher.h>
#include <ros/node_handle.h>
#else /* ROS2_BUILD */
#include <rclcpp/rclcpp.hpp>
#endif /* ROS2_BUILD */

// rosbag
#ifndef ROS2_BUILD
#include <rosbag/bag.h>
#else /* ROS2_BUILD */
#include <rosbag2_cpp/writer.hpp>
#endif /* ROS2_BUILD */

#ifndef ROS2_BUILD
// boost
#include <boost/shared_ptr.hpp>
#else /* ROS2_BUILD */
#include <memory>
#endif /* ROS2_BUILD */

namespace signal_logger_ros {

//! Log element for ros logging
template <typename ValueType_>
class LogElementRos: public signal_logger_std::LogElementStd<ValueType_>
{
  //! convinience typedefs
  using MsgType = typename traits::slr_msg_traits<ValueType_>::msgtype;
#ifndef ROS2_BUILD
  using MsgTypePtr = boost::shared_ptr<MsgType>;
#else /* ROS2_BUILD */
  using MsgTypePtr = std::shared_ptr<MsgType>;
#endif /* ROS2_BUILD */

 public:
  /** Constructor
   *  @param ptr        pointer to the log var
   *  @param bufferType buffer type of the log var
   *  @param bufferSize buffer size of the log var
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   *  @param headerStream string stream for log file header
   *  @param dataStream   type of the buffer
   *  @param nh           ros nodehandle for the ros publisher
   *  @param bagWriter    reference to the bagfile writer object
   *  @param saveToBag    flag if elemt shall be save to bag
   */
  LogElementRos(const ValueType_ * const ptr,
                const signal_logger::BufferType bufferType,
                const std::size_t bufferSize,
                const std::string & name,
                const std::string & unit,
                const std::size_t divider,
                const signal_logger::LogElementAction action,
                std::stringstream * headerStream,
                std::stringstream * dataStream,
#ifndef ROS2_BUILD
                ros::NodeHandle * nh,
                const std::shared_ptr<rosbag::Bag> & bagWriter
#else /* ROS2_BUILD */
                const rclcpp::Node::SharedPtr& nh,
                rosbag2_cpp::Writer& bagWriter
#endif /* ROS2_BUILD */
                ) :
                  signal_logger_std::LogElementStd<ValueType_>(ptr, bufferType, bufferSize, name, unit, divider, action, headerStream, dataStream),
                  nh_(nh),
                  bagWriter_(bagWriter),
                  wasPublished_(false)
  {
    msg_.reset(new MsgType());
    msgSave_.reset(new MsgType());
  }

  //! Destructor
  virtual ~LogElementRos()
  {

  }

  //! Save Data to file
  void saveDataToLogFile(const signal_logger::TimeElement & times,
                         unsigned int nrCollectDataCalls,
                         signal_logger::LogFileType type = signal_logger::LogFileType::BINARY) override
  {
    // If binary log file -> call super class
    if(type == signal_logger::LogFileType::BAG) {
      // Lock the copy mutex
      std::unique_lock<std::mutex> lock(this->mutexCopy_);

      // Oldest entry in the time buffer
      std::size_t startIdx = 0;

      if(this->bufferCopy_.getBufferType() == signal_logger::BufferType::LOOPING) {
        /* Last index of time: (times.size() - 1)
         * Index of newest time corresponding to a data point:  (nrCollectDataCalls - 1) % this->dividerCopy_
         * Offset of oldest time that corresponds to a data point: (this->bufferCopy_.size()-1) * this->dividerCopy_
         */
        startIdx = (times.getTimeBufferCopy().noTotalItems() - 1) - (nrCollectDataCalls - 1) % this->optionsCopy_.getDivider()
                   - (this->bufferCopy_.noTotalItems()-1) * this->optionsCopy_.getDivider();
      }

      const auto numTotalItems = this->bufferCopy_.noTotalItems();
      for(std::size_t i = 0; i < numTotalItems; ++i) {
        // Get time at data point
        signal_logger::TimestampPair tsp_now =
          times.getTimeBufferCopy().getElementCopyAtPosition((times.getTimeBufferCopy().noTotalItems() - 1) - (startIdx + i*this->optionsCopy_.getDivider()) );
#ifndef ROS2_BUILD
        ros::Time now = ros::Time(tsp_now.first, tsp_now.second);
#else /* ROS2_BUILD */
        builtin_interfaces::msg::Time now;
        now.sec = tsp_now.first;
        now.nanosec = tsp_now.second;
#endif /* ROS2_BUILD */
        // Update msg
        traits::slr_update_traits<ValueType_>::updateMsg(this->bufferCopy_.getPointerAtPosition( (numTotalItems - 1) - i), msgSave_.get(), now);
        // Write to bag
#ifndef ROS2_BUILD
        try{
          bagWriter_->write(this->optionsCopy_.getName(), now, *msgSave_);
        } catch(rosbag::BagException& exception) {
          MELO_ERROR_STREAM("[LogElementRos] Could not write to bag in element: " << this->getCopyOptions().getName());
          return;
        }
#else /* ROS2_BUILD */
        try{
          bagWriter_.write(*msgSave_, this->optionsCopy_.getName(), now);
        } catch(std::runtime_error& exception) {
          MELO_ERROR_STREAM("[LogElementRos] Could not write to bag in element: " << this->getCopyOptions().getName() << " error: " << exception.what());
          return;
        }
#endif /* ROS2_BUILD */
      }
    } else {
      signal_logger_std::LogElementStd<ValueType_>::saveDataToLogFile(times, nrCollectDataCalls, type);
    }
  }

  //! Reads buffer and publishes data via ros
  void publishData(const signal_logger::TimeElement & time, unsigned int nrCollectDataCalls) override
  {
    {
      std::unique_lock<std::mutex> lock(this->publishMutex_);
#ifndef ROS2_BUILD
      if (pub_.getNumSubscribers() == 0) {
#else /* ROS2_BUILD */
      if (pub_->get_subscription_count() == 0) {
#endif /* ROS2_BUILD */
        wasPublished_ = false;
        return;
      }
    }

    if(!wasPublished_) {
      wasPublished_ = true;
      this->buffer_.resetUnreadItems();
    }

    if(this->buffer_.noUnreadItems())
    {
      // Local vars
      signal_logger::TimestampPair tsp_now;
      ValueType_ data;

      {
        std::unique_lock<std::mutex> lock(this->mutex_);

        {
          std::unique_lock<std::mutex> timeLock(time.acquireMutex());

          // Define time index depending on buffer type
          std::size_t idx = (time.getBuffer().noTotalItems() - 1) - (this->buffer_.noTotalItems() -
          this->buffer_.noUnreadItems())*this->options_.getDivider();

          if(this->buffer_.getBufferType() == signal_logger::BufferType::LOOPING) {
            idx = (this->buffer_.noUnreadItems() - 1)*this->options_.getDivider() + (nrCollectDataCalls - 1) % this->options_.getDivider();
          }

          // get time stamp
          tsp_now = time.getTimeBuffer().getElementCopyAtPosition(idx);
        } // unlock time mutex

        // Read from buffer and transform to message via trait
        if(!this->buffer_.read(&data)) {
          return;
        }

      } // unlock elements mutex

      // publish over ros
#ifndef ROS2_BUILD
      traits::slr_update_traits<ValueType_>::updateMsg(&data, msg_.get(), ros::Time(tsp_now.first, tsp_now.second));
#else /* ROS2_BUILD */
      builtin_interfaces::msg::Time now;
      now.sec = tsp_now.first;
      now.nanosec = tsp_now.second;
      traits::slr_update_traits<ValueType_>::updateMsg(&data, msg_.get(), now);
#endif /* ROS2_BUILD */
      {
        std::unique_lock<std::mutex> lock(this->publishMutex_);
#ifndef ROS2_BUILD
        pub_.publish(msg_);
#else /* ROS2_BUILD */
        pub_->publish(*msg_);
#endif /* ROS2_BUILD */
      }

    }
  }

  //! Update the element, shutdown/advertise the ros publisher
  void update() override {
    std::unique_lock<std::mutex> lock(this->publishMutex_);
#ifndef ROS2_BUILD
    if(this->options_.isPublished() && this->isEnabled()) {
      pub_ = nh_->advertise<MsgType>(this->options_.getName(), 1);
    }  else {
      pub_.shutdown();
    }
#else /* ROS2_BUILD */
    if(this->options_.isPublished() && this->isEnabled()) {
      pub_ = nh_->create_publisher<MsgType>(this->options_.getName(), 1);
    }  else {
      pub_.reset();
    }
#endif /* ROS2_BUILD */
  }

  //! Cleanup the element (shutdown ros publisher)
  void cleanup() override {
    signal_logger_std::LogElementStd<ValueType_>::cleanup();
    std::unique_lock<std::mutex> lock(this->publishMutex_);
#ifndef ROS2_BUILD
    pub_.shutdown();
#else /* ROS2_BUILD */
    pub_.reset();
#endif /* ROS2_BUILD */
  }

 protected:
#ifndef ROS2_BUILD
  //! ros nodehandle
  ros::NodeHandle * nh_;
  //! bag writer
  const std::shared_ptr<rosbag::Bag> & bagWriter_;
  //! ros publisher
  ros::Publisher pub_;
#else /* ROS2_BUILD */
  //! ros nodehandle
  rclcpp::Node::SharedPtr nh_;
  //! bag writer
  rosbag2_cpp::Writer& bagWriter_;
  //! ros publisher
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
#endif /* ROS2_BUILD */
  //! published before
  std::atomic_bool wasPublished_;
  //! publisher mutex
  std::mutex publishMutex_;
  //! message pointer
  MsgTypePtr msg_;
  MsgTypePtr msgSave_;

};

} /* namespace signal_logger */
