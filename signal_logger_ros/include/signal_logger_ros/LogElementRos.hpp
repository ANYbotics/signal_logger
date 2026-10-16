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
#include <rclcpp/rclcpp.hpp>

// rosbag
#include <rosbag2_cpp/writer.hpp>

#include <memory>

namespace signal_logger_ros {

//! Log element for ros logging
template <typename ValueType_>
class LogElementRos: public signal_logger_std::LogElementStd<ValueType_>
{
  //! convinience typedefs
  using MsgType = typename traits::slr_msg_traits<ValueType_>::msgtype;
  using MsgTypePtr = std::shared_ptr<MsgType>;

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
                const rclcpp::Node::SharedPtr& nh,
                rosbag2_cpp::Writer& bagWriter
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
        builtin_interfaces::msg::Time now;
        now.sec = tsp_now.first;
        now.nanosec = tsp_now.second;
        // Update msg
        traits::slr_update_traits<ValueType_>::updateMsg(this->bufferCopy_.getPointerAtPosition( (numTotalItems - 1) - i), msgSave_.get(), now);
        // Write to bag
        try{
          bagWriter_.write(*msgSave_, this->optionsCopy_.getName(), now);
        } catch(std::runtime_error& exception) {
          MELO_ERROR_STREAM("[LogElementRos] Could not write to bag in element: " << this->getCopyOptions().getName() << " error: " << exception.what());
          return;
        }
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
      if (pub_->get_subscription_count() == 0) {
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
      builtin_interfaces::msg::Time now;
      now.sec = tsp_now.first;
      now.nanosec = tsp_now.second;
      traits::slr_update_traits<ValueType_>::updateMsg(&data, msg_.get(), now);
      {
        std::unique_lock<std::mutex> lock(this->publishMutex_);
        pub_->publish(*msg_);
      }

    }
  }

  //! Update the element, shutdown/advertise the ros publisher
  void update() override {
    std::unique_lock<std::mutex> lock(this->publishMutex_);
    if(this->options_.isPublished() && this->isEnabled()) {
      pub_ = nh_->create_publisher<MsgType>(this->options_.getName(), 1);
    }  else {
      pub_.reset();
    }
  }

  //! Cleanup the element (shutdown ros publisher)
  void cleanup() override {
    signal_logger_std::LogElementStd<ValueType_>::cleanup();
    std::unique_lock<std::mutex> lock(this->publishMutex_);
    pub_.reset();
  }

 protected:
  //! ros nodehandle
  rclcpp::Node::SharedPtr nh_;
  //! bag writer
  rosbag2_cpp::Writer& bagWriter_;
  //! ros publisher
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  //! published before
  std::atomic_bool wasPublished_;
  //! publisher mutex
  std::mutex publishMutex_;
  //! message pointer
  MsgTypePtr msg_;
  MsgTypePtr msgSave_;

};

} /* namespace signal_logger */
