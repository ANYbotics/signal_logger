/*
 * SignalLoggerBase.cpp
 *
 *  Created on: Sep 28, 2016
 *      Author: Gabriel Hottiger
 */

// signal logger
#include "signal_logger/SignalLoggerBase.hpp"

// stl
#include "assert.h"
#include <fstream>
#include <thread>
#include <ctime>
#include <ratio>
#include <chrono>

namespace signal_logger {

SignalLoggerBase::SignalLoggerBase():
      isInitialized_(false),
      isUpdateLocked_(false),
      isCollectingData_(false),
      noCollectDataCalls_(0),
      noDataInBuffer_(0),
      bufferStorageRate_(0),
      bufferSize_(0),
      collectScriptFileName_(std::string{LOGGER_DEFAULT_SCRIPT_FILENAME}),
      updateFrequency_(0),
      samplingFrequency_(0),
      samplingTime_(0.0),
      logElements_(),
      loggerMutex_(),
      scriptMutex_()
{

}

SignalLoggerBase::~SignalLoggerBase()
{

}

void SignalLoggerBase::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& collectScriptFileName)
{
  // Assert corrupted configuration
  assert(samplingFrequency <= updateFrequency);
  assert(samplingFrequency > 0);
  assert(updateFrequency > 0);

  // Lock mutex until logger is initialized
  std::lock_guard<std::mutex> lockLogger(loggerMutex_);

  // Set configuration
  updateFrequency_ = updateFrequency;
  samplingFrequency_ = samplingFrequency;
  samplingTime_ = samplingTime;
  bufferStorageRate_ = static_cast<double>(updateFrequency_) / static_cast<double>(samplingFrequency_);
  bufferSize_ = samplingTime_ * static_cast<double>(samplingFrequency_);
  collectScriptFileName_ = collectScriptFileName;

  // Notify user
  MELO_INFO("Signal Logger was initialized!");
  isInitialized_ = true;
}

void SignalLoggerBase::startLogger()
{
  if(!isInitialized_)
  {
    MELO_WARN("Signal logger could not be started! Logger not initialized.");
    return;
  }

  isCollectingData_ = true;
  noCollectDataCalls_ = 0;
  noDataInBuffer_ = 0;
}

void SignalLoggerBase::stopLogger()
{
  if(!isInitialized_ || !isCollectingData_)
  {
    MELO_WARN("Signal logger could not be stopped!");
    return;
  }

  isCollectingData_ = false;
}

void SignalLoggerBase::restartLogger()
{
  this->stopLogger();
  this->startLogger();
}

void SignalLoggerBase::updateLogger(bool updateScript) {

  if(!isInitialized_ || isCollectingData_ || isUpdateLocked_)
  {
    MELO_WARN("Signal logger could not be updated!");
    return;
  }

  // update logging script
  if (updateScript) {
    std::lock_guard<std::mutex> lockScript(scriptMutex_);

    std::ofstream collectScript;
    collectScript.open(collectScriptFileName_, std::ios::out | std::ios::trunc);

    if(collectScript.is_open())
    {
      for(auto & elem : logElements_)
      {
        collectScript << elem.second->getName() << std::endl;
      }
    }
    else {
      MELO_ERROR("Could not open script file %s ", collectScriptFileName_.c_str());
      return;
    }

    // close file
    collectScript.close();
  }

  readDataCollectScript(collectScriptFileName_);
}

void SignalLoggerBase::lockUpdate()
{
  isUpdateLocked_ = true;
}

void SignalLoggerBase::collectLoggerData()
{
  if(!isInitialized_) return;

  // Is logger started?
  if(isCollectingData_ && noCollectDataCalls_%bufferStorageRate_ == 0)
  {
    // Add data to buffer
    for(auto & elem : logElements_)
    {
      if(elem.second->isEnabled())
      {
        elem.second->collectData();
      }
    }
    // There can not be more data in the buffer that the bufferSize is
    noDataInBuffer_ = std::min(static_cast<unsigned int>(bufferSize_), ++noDataInBuffer_);
  }

  // #TODO handle overflow? -> is it a problem?
  ++noCollectDataCalls_;
}

void SignalLoggerBase::saveLoggerData()
{
  if(!isInitialized_)
  {
    MELO_WARN("Signal logger could not save data!");
    return;
  }

  // Get local time
  std::time_t now = std::chrono::system_clock::to_time_t ( std::chrono::system_clock::now() );
  std::tm now_loc = *std::localtime(&now);

  // Filename format (e.g. d_13Sep2016_12-13-49)
  char filename[21];
  strftime(filename, sizeof filename, "d_%d%b%Y_%H-%M-%S", &now_loc);

  // Save data in different thread
  std::thread t1(&SignalLoggerBase::workerSaveData, this,  std::string{filename});
  t1.detach();
}

void SignalLoggerBase::stopAndSaveLoggerData()
{
  this->stopLogger();
  this->saveLoggerData();
}

bool SignalLoggerBase::readDataCollectScript(const std::string & scriptName)
{

  if (!isInitialized_) {
    return false;
  }

  // Lock script
  std::lock_guard<std::mutex> lockScript(scriptMutex_);

  std::ifstream collectScript(scriptName);

  if(collectScript.is_open())
  {
    if(isCollectingData_) {
      // Logger can not run when reading script
      // #TODO inform user
      stopLogger();
    }

    // Disable all log data and reallocate buffer
    for(auto & elem : logElements_)
    {
      elem.second->setIsEnabled(false);
    }

    // Read logging script and enable present data
    std::string readName;
    while(std::getline(collectScript, readName))
    {
      if(logElements_.find(readName) != logElements_.end())
      {
        // Enable item collection
        logElements_.at(readName)->setIsEnabled(true);
      }
      else {
        MELO_WARN("Can not log data! Data with name %s was not added to the logger. ", readName.c_str());
      }
    }
  }
  else {
    MELO_ERROR("Could not open script file %s ", scriptName.c_str());
    return false;
  }

  // close script
  collectScript.close();

  return true;
}


}


