/*
 * SignalLoggerExample.hpp
 *
 *  Created on: May 18, 2017
 *      Author: Gabriel Hottiger
 */
#pragma once

// nodewrap
#include "any_node/Node.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// ros
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

// Boost
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// STL
#include <atomic>
#include <thread>
#include <signal.h>
#include <stdlib.h>

namespace signal_logger_example {

class SignalLoggerExample: public any_node::Node
{
  public:
    SignalLoggerExample(NodeHandlePtr nh);

    virtual void init();
    virtual void cleanup();
    virtual bool update(const any_worker::WorkerEvent& event);
    virtual void publishWorker();
    virtual void readWorker();

  private:
    std::thread publishThread_;
    std::thread readThread_;
    std::atomic_bool shouldPublish_;
    std::atomic_bool shouldRead_;
    double logVar_;
    ros::Time time_;

};

} /* namespace m545_tf_publisher */
