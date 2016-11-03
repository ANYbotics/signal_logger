/*
 * signal_logger_std.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <signal_logger/signal_logger.hpp>
#include <signal_logger_std/SignalLoggerStd.hpp>

namespace signal_logger {

void addSignalLoggerStd() {
  logger.reset(new signal_logger_std::SignalLoggerStd() );
}

}
