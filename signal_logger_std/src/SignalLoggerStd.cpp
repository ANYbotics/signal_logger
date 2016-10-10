/*
 * SignalLoggerStd.cpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#include "signal_logger_std/SignalLoggerStd.hpp"

namespace signal_logger_std {

SignalLoggerStd::SignalLoggerStd():
        signal_logger::SignalLoggerBase(),
        file_(),
        headerStream_(std::ostringstream::in | std::ostringstream::out),
        dataStream_(std::ios::in | std::ios::out | std::ios::binary)
{

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
  for(auto & elem : logElements_) {
    if(elem.second->isEnabled() && (elem.second->getAction() == signal_logger::LogElementInterface::LogElementAction::SAVE ||
       elem.second->getAction() == signal_logger::LogElementInterface::LogElementAction::SAVE_AND_PUBLISH) )
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
  file_ << dataStream_.rdbuf() << std::endl;

  // Close file
  file_.close();

  sleep(3);

  isSavingData_ = false;
  MELO_INFO( "All done, captain!" );

  return true;

}

} /* namespace signal_logger */
