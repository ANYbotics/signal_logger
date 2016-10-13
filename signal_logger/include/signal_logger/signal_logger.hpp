/**
* @file 	  signal_logger.hpp
* @author 	  Christian Gehring, C. Dario Bellicoso, Gabriel Hottiger
* @date		  June 26, 2013
*/

#pragma once

#ifdef E
#undef E
#endif

#include "signal_logger/SignalLoggerBase.hpp"
#include <memory>

namespace signal_logger {

//! Reference to the logger
extern std::shared_ptr<SignalLoggerBase> logger;

} // end namespace
