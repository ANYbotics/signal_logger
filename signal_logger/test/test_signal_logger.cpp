/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 */

#include <gtest/gtest.h>

#include "signal_logger/signal_logger.hpp"

// This is not really a test but merely a driver to play around with heaptrack
TEST(TestSignalLogger, testMemoryUsage) {  // NOLINT

  // Something to log
  kindr::Velocity3D value{-1., -2., -3.};

  std::this_thread::sleep_for(std::chrono::seconds{1});

  // Setup signal logger
  signal_logger::setSignalLoggerStd();
  signal_logger::SignalLoggerOptions options;
  options.maxLoggingTime_ = 10.;
  options.updateFrequency_ = 100;
  options.loggerPrefix_ = "";
  options.collectScriptFileName_ = "silo_example_script.yaml";

  std::this_thread::sleep_for(std::chrono::seconds{1});

  signal_logger::logger->initLogger(options);
  signal_logger::add(value, "localAngularVelocity");
  signal_logger::logger->updateLogger();
  signal_logger::logger->setName("testMemoryLeaks");
  signal_logger::logger->startLogger();

  std::this_thread::sleep_for(std::chrono::seconds{1});

  // Collect first sample
  signal_logger::logger->collectLoggerData();

  std::this_thread::sleep_for(std::chrono::seconds{1});

  // Simulate changing of data and log again
  value.setRandom();
  signal_logger::logger->collectLoggerData();

  std::this_thread::sleep_for(std::chrono::seconds{1});

  signal_logger::logger->stopAndSaveLoggerData({signal_logger::LogFileType::BINARY});

  std::this_thread::sleep_for(std::chrono::seconds{1});

  signal_logger::logger->cleanup();

  std::this_thread::sleep_for(std::chrono::seconds{1});

}
