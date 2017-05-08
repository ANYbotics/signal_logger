/*!
 * @file     LogElementOptions.hpp
 * @author   Gabriel Hottiger
 * @date     May 05, 2017
 * @brief    Log element options class.
 */

#pragma once

// Signal logger
#include "signal_logger_core/typedefs.hpp"

// STL
#include <atomic>

namespace signal_logger {

//! Forward declare options
class LogElementOptions;

class LogElementOptions
{
 public:
  /** Constructor
   *  @param name       name of the log var
   *  @param unit       unit of the log var
   *  @param divider    log_freq = ctrl_freq/divider
   *  @param action     save, publish or save and publish
   */
  LogElementOptions(const std::string & name,
                    const std::string & unit,
                    const std::size_t divider,
                    const LogElementAction action) :
     name_(name),
     unit_(unit),
     divider_(divider),
     action_(action),
     isEnabled_(false)
  {

  }

  std::string getName() const { return name_; }

  std::string getUnit() const { return unit_; }

  std::size_t getDivider() const { return divider_.load(); }
  void setDivider(const std::size_t divider) { divider_.store(divider); }

  LogElementAction getAction() const { return action_.load(); }
  // todo update element on action change
  void setAction(const LogElementAction action) { action_.store(action); }

  bool isEnabled() const { return isEnabled_.load(); }
  // todo allocate memory if newly enabled and update
  void setIsEnabled(const bool isEnabled) { isEnabled_.store(isEnabled); }

  //! @return check action for publishing
  bool isPublished() const {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::PUBLISH);
  }

  //! @return check action for saving
  bool isSaved() const {
    return (action_ == LogElementAction::SAVE_AND_PUBLISH || action_ == LogElementAction::SAVE);
  }

 private:
  //! Name of the log element
  const std::string name_;
  //! Unit of the log element
  const std::string unit_;
  //! Defines log element collection frequency = updateFrequency/divider
  std::atomic_size_t divider_;
  //! Action
  std::atomic<LogElementAction> action_;
  //! Indicates if log element is currently active
  std::atomic_bool isEnabled_;
};

} /* namespace signal_logger */
