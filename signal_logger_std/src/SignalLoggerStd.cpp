/*
 * SignalLoggerStd.cpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#include "signal_logger_std/SignalLoggerStd.hpp"

namespace signal_logger_std {

SignalLoggerStd::SignalLoggerStd(const std::string & loggerPrefix):
        signal_logger::SignalLoggerBase(loggerPrefix),
        file_(),
        headerStream_(std::ostringstream::in | std::ostringstream::out),
        dataStream_(std::ios::in | std::ios::out | std::ios::binary)
{
  timeElement_.reset(new signal_logger_std::LogElementStd<signal_logger::TimestampPair>( &logTime_, loggerPrefix + std::string{"/time"},
                                                                                         "[s/ns]", 1, signal_logger::LogElementAction::SAVE,
                                                                                         0, signal_logger::BufferType::EXPONENTIALLY_GROWING,
                                                                                         &headerStream_, &dataStream_));
}

SignalLoggerStd::~SignalLoggerStd()
{

}

//! Save all the buffered data into a log file
bool SignalLoggerStd::workerSaveData(const std::string & logFileName) {

  // Clear streams
  headerStream_.str(std::string());
  dataStream_.str(std::string());

  // Fill streams
  headerStream_ << "// Name SizeInBytes NrData Divider LoopingBuffer(0 = false, 1 = true)" << std::endl;
  timeElement_->saveDataToLogFile();

  for(auto & elem : logElements_) {
    if(elem.second->isEnabled() && elem.second->isSaved())
    {
      elem.second->saveDataToLogFile();
      elem.second->clearBuffer();
    }
  }

  // Write string header
  file_.open(logFileName, std::ios::out | std::ios::trunc);
  file_ << headerStream_.str() << std::endl;
  file_.close();

  // Write binary data
  file_.open(logFileName, std::ios::out | std::ios::app | std::ios::binary);
  file_ << dataStream_.rdbuf();

  // Close file
  file_.close();

  // Set flag #Fixme this should not be responsibility of the subclass!
  isSavingData_ = false;
  MELO_INFO( "All done, captain!" );

  return true;
}

} /* namespace signal_logger_std */
