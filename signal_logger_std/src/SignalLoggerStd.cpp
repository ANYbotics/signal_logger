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

  // Open file
  file_.open(logFileName, std::ios::out | std::ios::trunc);
  file_ << "// DataName noBytes noPts divider" << std::endl;

  for(auto & elem : logElements_) {
    if(elem.second->isEnabled()) {
      elem.second->writeHeaderToLogFile();
    }
  }

  //    # FIXME for binary
  file_.close();
  file_.open(logFileName, std::ios::out | std::ios::app | std::ios::binary);
  for(auto & elem : logElements_) {
    if(elem.second->isEnabled()) {
      elem.second->writeDataToLogFile();
    }
  }

  file_.close();

  MELO_INFO( "All done, captain!" );

  return true;

}

} /* namespace signal_logger */
