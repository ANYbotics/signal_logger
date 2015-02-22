/*
 * LogElementBase.hpp
 *
 *  Created on: Feb 21, 2015
 *      Author: C. Dario Bellicoso
 */

#ifndef LOGELEMENTBASE_HPP_
#define LOGELEMENTBASE_HPP_

#include <ros/ros.h>
#include <string>

namespace signal_logger_ros {

class LogElementBase {
 public:
  LogElementBase();
  virtual ~LogElementBase();

  virtual const std::string& getTopicName() = 0;
  virtual void publish(const ros::Time& timeStamp) = 0;
  virtual const uint32_t getNumSubscribers() const = 0;
};

} /* namespace signal_logger_ros */

#endif /* LOGELEMENTBASE_HPP_ */
