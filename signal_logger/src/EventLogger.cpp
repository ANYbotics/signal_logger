#include "signal_logger/EventLogger.hpp"

#include <message_logger/message_logger.hpp>

namespace signal_logger {

EventLogger::EventLogger(ros::NodeHandle& nodeHandle, const std::string& name, const std::string& configFile, double timeBefore,
                         double timeAfter, double minTriggerGap, int updateFrequency)
    : nodeHandle_(nodeHandle),
      signalLogger_(std::make_unique<signal_logger_ros::SignalLoggerRos>(&nodeHandle_)),
      timeAfter_(timeAfter),
      minTriggerGap_(minTriggerGap),
      bufferSize_(static_cast<int>((timeBefore + timeAfter) * static_cast<double>(updateFrequency))) {
  try {
    acl_time::constructGlobalDynamicClock(nodeHandle);
  } catch (const std::runtime_error& e) {
    MELO_INFO("Global dynamic clock already constructed")
  }
  if (!acl_time::waitForGlobalDynamicClockInitialization(acl_time::Duration(120.0))) {
    throw std::runtime_error("Failed to initialize global dynamic clock.");
  }
  signal_logger::SignalLoggerOptions siloOptions;
  siloOptions.updateFrequency_ = updateFrequency;
  siloOptions.maxLoggingTime_ = timeBefore + timeAfter;
  siloOptions.collectScriptFileName_ = configFile;
  siloOptions.loggerPrefix_ = nodeHandle_.getNamespace() + "/silo";
  signalLogger_->initLogger(siloOptions);
  signalLogger_->setName(name);
}

EventLogger::~EventLogger() {
  stop();
  signalLogger_->cleanup();
}

void EventLogger::start() {
  signalLogger_->startLogger();
}

void EventLogger::stop() {
  dumpIfTimerPending();
  cleanUp();
  signalLogger_->stopLogger();
}

void EventLogger::collectData() {
  signalLogger_->collectLoggerData();
}

void EventLogger::dump() {
  signalLogger_->saveLoggerData(logFileTypes_);
  cleanUp();
}

void EventLogger::triggerDump() {
  std::scoped_lock<std::mutex> scopedLock(timersMutex_);

  // Check if time gap between last and current event is large enough.
  bool trigger{true};
  const auto now{acl_time::GlobalDynamicClock::now()};
  for (const auto& timer : timers_) {
    if (now - timer.triggered < acl_time::Duration(minTriggerGap_)) {
      trigger = false;
      break;
    }
  }

  // Add timer to save logs to file in the future.
  if (trigger) {
    timers_.push_back({now, nodeHandle_.createTimer(
                                ros::Duration(timeAfter_), [this](const ros::TimerEvent&) { dump(); }, true)});
  } else {
    MELO_INFO_STREAM("Logger dump was triggered to close to previous dump.")
  }
}

void EventLogger::dumpIfTimerPending() {
  bool pending{false};
  {
    std::scoped_lock<std::mutex> scopedLock(timersMutex_);
    const auto now{acl_time::GlobalDynamicClock::now()};
    for (auto& timer : timers_) {
      if (now - timer.triggered < acl_time::Duration(timeAfter_)) {
        timer.timer.stop();
        pending = true;
      }
    }
  }
  if (pending) {
    dump();
  }
}

void EventLogger::publish() {
  signalLogger_->publishData();
}

void EventLogger::cleanUp() {
  std::scoped_lock<std::mutex> scopedLock(timersMutex_);

  // Remove finished timers.
  auto it = timers_.begin();
  while (it != timers_.end()) {
    if (!it->timer.hasPending()) {
      it = timers_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace signal_logger
