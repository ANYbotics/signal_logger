/*
 * LogElement.hpp
 *
 *  Created on: Feb 21, 2015
 *      Author: C. Dario Bellicoso
 */

#ifndef LOGELEMENT_HPP_
#define LOGELEMENT_HPP_

#include "signal_logger_ros/LogElementBase.hpp"
#include "signal_logger_ros/signal_logger_ros_traits.hpp"

#include <ros/ros.h>
#include <boost/any.hpp>

#include "realtime_tools/realtime_publisher.h"

namespace signal_logger_ros {


const int DEFAULT_QUEUE_SIZE = 0;

// generic declaration
template <typename LogType_, bool vectorAtPosition = false>
class LogElement;

// partial specialization
template <typename LogType_>
class LogElement<LogType_, false> : public LogElementBase {
 public:

  typedef traits::slr_traits<LogType_> Traits;

  LogElement(ros::NodeHandle& nodeHandle,
             const std::string& name,
             const LogType_* varPtr) :
    topicName_(name),
    vectorPtr_(varPtr)
  {
//    rtPub_ = new realtime_tools::RealtimePublisher<typename Traits::msgtype>(nodeHandle, name, DEFAULT_QUEUE_SIZE);
    pub_ = nodeHandle.advertise<typename Traits::msgtype>(topicName_, DEFAULT_QUEUE_SIZE);
  }

  ~LogElement() {
//    pub_.shutdown();
//    delete rtPub_;
  }

  virtual const uint32_t getNumSubscribers() const {
//    return rtPub_->getNumSubscribers();
    return pub_.getNumSubscribers();
  }

  virtual void setLogVarPointer(const LogType_* varPtr) {
    vectorPtr_ = varPtr;
  }

  virtual const std::string& getTopicName() {
    return topicName_;
  }

  virtual void publish(const ros::Time& timeStamp) {
    typename Traits::msgtype msg;
    Traits::updateMsg(vectorPtr_, msg, timeStamp);
//    if (rtPub_->trylock()) {
//      rtPub_->msg_ = msg;
//      rtPub_->unlockAndPublish();
//    }
    pub_.publish(msg);
  }

 private:
  std::string topicName_;
  const LogType_* vectorPtr_;
//  realtime_tools::RealtimePublisher<typename Traits::msgtype>* rtPub_;
  ros::Publisher pub_;
};


// partial specialization
template <typename LogType_>
class LogElement<LogType_, true> : public LogElementBase {
 public:

  typedef traits::slr_traits<LogType_, true> Traits;

  LogElement(ros::NodeHandle& nodeHandle,
             const std::string& topic,
             const LogType_* varPtr,
             const signal_logger::LoggerBase::KindrPositionD* positionPtr,
             const std::string& vectorFrame,
             const std::string& positionFrame,
             const std::string& name
             ) :
    topicName_(topic),
    vectorPtr_(varPtr),
    positionPtr_(positionPtr),
    vectorFrameId_(vectorFrame),
    positionFrameId_(positionFrame),
    labelName_(name)
  {
//    rtPub_ = new realtime_tools::RealtimePublisher<kindr_msgs::VectorAtPosition>(nodeHandle, topic, DEFAULT_QUEUE_SIZE);
    pub_ = nodeHandle.advertise<typename Traits::msgtype>(topicName_, DEFAULT_QUEUE_SIZE);
  }

  ~LogElement() {
//    pub_.shutdown();
//    delete rtPub_;
  }

  virtual const uint32_t getNumSubscribers() const {
//    return rtPub_->getNumSubscribers();
    return pub_.getNumSubscribers();
  }

  virtual void setLogVarPointer(const LogType_* varPtr) {
    vectorPtr_ = varPtr;
  }

  virtual void setLogVarAtPositionPointer(const LogType_* varPtr,
                                          const signal_logger::LoggerBase::KindrPositionD* pos,
                                          const std::string& vectorFrame,
                                          const std::string& positionFrame,
                                          const std::string& name) {
    vectorPtr_ = varPtr;
    positionPtr_ = pos;
    vectorFrameId_ = vectorFrame;
    positionFrameId_ = positionFrame;
    labelName_ = name;
  }

  virtual const std::string& getTopicName() {
    return topicName_;
  }

  virtual void publish(const ros::Time& timeStamp) {
    typename Traits::msgtype msg;
    Traits::updateMsg(vectorPtr_,
                      positionPtr_,
                      vectorFrameId_,
                      positionFrameId_,
                      labelName_,
                      msg,
                      timeStamp);
//    if (rtPub_->trylock()) {
//      rtPub_->msg_ = msg;
//      rtPub_->unlockAndPublish();
//    }
    pub_.publish(msg);
  }

 private:
  std::string topicName_;
  std::string labelName_;
  std::string vectorFrameId_;
  std::string positionFrameId_;

  kindr_msgs::VectorAtPosition::Type kindrMsgType_;

  const LogType_* vectorPtr_;
  const signal_logger::LoggerBase::KindrPositionD* positionPtr_;

//  realtime_tools::RealtimePublisher<typename Traits::msgtype>* rtPub_;
  ros::Publisher pub_;
};


} /* namespace signal_logger_ros */

#endif /* LOGELEMENT_HPP_ */
