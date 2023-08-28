#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <ros/node_handle.h>
#include <ros/timer.h>

#include <acl_time/acl_time.hpp>
#include <signal_logger/signal_logger.hpp>

namespace signal_logger {

/**
 * The timer.
 */
struct Timer {
  Timer(const acl_time::DynamicTimePoint& triggered, const ros::Timer& timer) : timer(timer), triggered(triggered) {}
  //! Ros timer.
  ros::Timer timer;
  //! Time point when the one-shot timer was started.
  //! This is used to check if the minimal gap between events is ok.
  acl_time::DynamicTimePoint triggered;
};

/**
 * The event logger.
 * Logs data before and after an event and writes it eventually to a file.
 */
class EventLogger {
 public:
  /**
   * Constructor.
   * @param nodeHandle The Ros node handle.
   * @param name The logger name.
   * @param configFile The config file name.
   * @param timeBefore The time [s] to log before the event.
   * @param timeAfter The time [s] to log after the event.
   * @param minTriggerGap The minimal gap between events.
   * @param updateFrequency The update frequency [Hz].
   */
  EventLogger(ros::NodeHandle& nodeHandle, const std::string& name, const std::string& configFile, double timeBefore,
              double timeAfter, double minTriggerGap, int updateFrequency);

  /**
   * Destructor.
   */
  ~EventLogger();

  EventLogger(const EventLogger&) = delete;
  EventLogger& operator=(EventLogger const&) = delete;
  EventLogger(EventLogger&&) = delete;
  EventLogger& operator=(EventLogger&&) = delete;

  /**
   * Add a variable to the log.
   * @tparam Type The type of the variable.
   * @param variable The variable.
   * @param name The name of the variable.
   * @param group The group of the variable.
   * @param unit The unit of the variable.
   * @param action The log action (safe and/or publish).
   */
  template <typename Type>
  void addVariable(const Type* const variable, const std::string& name, const std::string& group, const std::string& unit,
                   const signal_logger::LogElementAction action) {
    dynamic_cast<signal_logger_ros::SignalLoggerRos*>(signalLogger_.get())
        ->add(variable, name, group, unit, 1, action, bufferSize_, signal_logger::BufferType::LOOPING);
    signalLogger_->updateLogger();
  }

  /**
   * Start logger.
   */
  void start();

  /**
   * Stop logger.
   */
  void stop();

  /**
   * Collect data.
   */
  void collectData();

  /**
   * Save to file.
   */
  void dump();

  /**
   * Trigger save to file to run in the future.
   */
  void triggerDump();

  /**
   * Save log to file if there is at least one timer pending.
   * This can be used for shutdown when waiting for the timer is not an option.
   */
  void dumpIfTimerPending();

  /**
   * Publish the data.
   */
  void publish();

 private:
  /**
   * Clean up finished timers.
   */
  void cleanUp();

  //! The Ros node handle.
  ros::NodeHandle nodeHandle_;

  //! The signal logger.
  std::unique_ptr<signal_logger::SignalLoggerBase> signalLogger_;
  //! The log file type.
  const signal_logger::LogFileTypeSet logFileTypes_{signal_logger::LogFileType::BINARY};

  //! The time in the future to save logs to file.
  const double timeAfter_{10.0};
  //! The minimal gap between dump triggers.
  const double minTriggerGap_{5.0};
  //! The buffer size.
  const int bufferSize_{1};

  //! Thread safe access to timers.
  std::mutex timersMutex_;
  //! The timers.
  std::vector<Timer> timers_;
};

}  // namespace anymal_low_level_controller
