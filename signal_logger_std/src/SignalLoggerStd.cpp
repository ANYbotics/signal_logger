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
        file_()
{

}

SignalLoggerStd::~SignalLoggerStd()
{

}

//! Save all the buffered data into a log file
bool SignalLoggerStd::workerSaveData(const std::string & logFileName) {

  // Clear streams
  headerStream_.clear();
  dataStream_.clear();

  // Fill streams
  for(auto & elem : logElements_) {
    if(elem.second->isEnabled()) {
      elem.second->saveDataToLogFile();
    }
  }

  // Write string header
  file_.open(logFileName, std::ios::out | std::ios::trunc);
  file_ << headerStream_ << std::endl;

  // Write binary data
  file_.open(logFileName, std::ios::out | std::ios::app | std::ios::binary);
  file_ << dataStream_ << std::endl;

  // Close file
  file_.close();
  MELO_INFO( "All done, captain!" );

  return true;

}

} /* namespace signal_logger */
