/*
 * SignalLoggerBase.cpp
 *
 *  Created on: Sep 28, 2016
 *      Author: Gabriel Hottiger
 */

// signal logger
#include "signal_logger/SignalLoggerBase.hpp"

// yaml
#include <yaml-cpp/yaml.h>

// boost
#include <boost/filesystem.hpp>

// stl
#include "assert.h"
#include <sys/stat.h>
#include <thread>
#include <ctime>
#include <ratio>
#include <chrono>
#include <fstream>

namespace signal_logger {

SignalLoggerBase::SignalLoggerBase():
                              isInitialized_(false),
                              isUpdateLocked_(false),
                              isCollectingData_(false),
                              isSavingData_(false),
                              noCollectDataCalls_(0),
                              collectScriptFileName_(std::string{LOGGER_DEFAULT_SCRIPT_FILENAME}),
                              updateFrequency_(0),
                              logElements_(),
                              scriptMutex_()
{

}

SignalLoggerBase::~SignalLoggerBase()
{

}

void SignalLoggerBase::initLogger(int updateFrequency, const std::string& collectScriptFileName)
{
  // Assert corrupted configuration
  assert(updateFrequency > 0);

  // Set configuration
  updateFrequency_ = updateFrequency;
  collectScriptFileName_ = collectScriptFileName;

  // Notify user
  MELO_INFO("Signal Logger was initialized!");

  isInitialized_ = true;
}

bool SignalLoggerBase::startLogger()
{
  if(!isInitialized_ || isSavingData_ || isCollectingData_)
  {
    MELO_WARN("Signal logger could not be started!%s%s%s", !isInitialized_?" Not initialized!":"",
        isSavingData_?" Saving data!":"", isCollectingData_?" Already running!":"");
    return false;
  }

  // Reset flags and data collection calls
  isCollectingData_ = true;
  noCollectDataCalls_ = 0;

  return true;
}

bool SignalLoggerBase::stopLogger()
{
  if(!isInitialized_ || !isCollectingData_)
  {
    MELO_WARN("Signal logger could not be stopped!%s%s", !isInitialized_?" Not initialized!":"",
        !isCollectingData_?" Not running!":"");
    return false;
  }

  isCollectingData_ = false;

  return true;
}

bool SignalLoggerBase::restartLogger()
{
  return this->stopLogger() && this->startLogger();
}

bool SignalLoggerBase::updateLogger(bool updateScript) {

  if(isUpdateLocked_ || !isInitialized_ || isCollectingData_ || isSavingData_)
  {
    MELO_WARN("Signal logger could not be updated!%s%s%s%s", isUpdateLocked_?" Update locked!":"",
        !isInitialized_?" Not initialized!":"", isSavingData_?" Saving data!":"", isCollectingData_?" Collecting data!":"");
    return false;
  }

  if(updateScript) {
    // Enable all log data to update script
    for(auto & elem : logElements_)
    {
      elem.second->setIsEnabled(true);
    }
    saveDataCollectScript(collectScriptFileName_);
  }

  return readDataCollectScript(collectScriptFileName_);

}

void SignalLoggerBase::lockUpdate(bool lock)
{
  isUpdateLocked_ = lock;
}

bool SignalLoggerBase::collectLoggerData()
{
  if(!isInitialized_ || isSavingData_) return false;

  // Is logger started?
  if(isCollectingData_)
  {
    // Add data to buffer
    for(auto & elem : logElements_)
    {
      if(elem.second->isEnabled() && (noCollectDataCalls_ % elem.second->getDivider()) == 0)
      {
        elem.second->collectData();
      }
    }
  }

  ++noCollectDataCalls_;

  return true;
}

bool SignalLoggerBase::publishData()
{
  // Publish data from buffer
  for(auto & elem : logElements_)
  {
    if(elem.second->isEnabled() && elem.second->isPublished())
    {
      elem.second->publishData();
    }
  }

  return true;
}

bool SignalLoggerBase::saveLoggerData()
{
  isSavingData_ = true;

  if(!isInitialized_)
  {
    MELO_WARN("Signal logger could not save data!");
    isSavingData_ = false;
    return false;
  }

  // Read suffix number from file
  int suffixNumber = 0;
  std::ifstream ifs(".last_data", std::ifstream::in);
  if(ifs.is_open())
  {
    ifs >> suffixNumber;
  }
  ifs.close();

  // Update suffix number
  ++suffixNumber;

  // Write next suffix number to file
  std::ofstream ofs(".last_data", std::ofstream::out | std::ofstream::trunc);
  if(ofs.is_open())
  {
    ofs << suffixNumber;
  }
  ofs.close();

  // To string
  std::string suffixString = std::to_string(suffixNumber);
  while(suffixString.length() != 5 ) { suffixString.insert(0, "0"); }

//  // Check for file existance
//  const boost::filesystem::path currentDir( boost::filesystem::current_path() );
//  const boost::filesystem::directory_iterator end;
//  boost::filesystem::directory_iterator it;
//  std::string checkString;
//  int i = -1;
//  do{
//    checkString = std::string{"log_"} + std::to_string(++i);
//    it = std::find_if(boost::filesystem::directory_iterator(currentDir), end,
//                      [&checkString](const boost::filesystem::directory_entry& e) {
//      return e.path().filename().string().compare(0,checkString.size(),checkString) == 0; });
//  } while(it != end);

  // Get local time
  std::time_t now = std::chrono::system_clock::to_time_t ( std::chrono::system_clock::now() );
  std::tm now_loc = *std::localtime(&now);
  char dateTime[21];
  strftime(dateTime, sizeof dateTime, "%Y%m%d_%H-%M-%S_", &now_loc);

  // Filename format (e.g. d_13Sep2016_12-13-49) add nr
  std::string filename = std::string{"d_"} + std::string{dateTime} + suffixString;

  // Save data in different thread
  std::thread t1(&SignalLoggerBase::workerSaveData, this,  std::string{filename});
  t1.detach();

  return true;
}

bool SignalLoggerBase::stopAndSaveLoggerData()
{
  return this->stopLogger() && this->saveLoggerData();
}

bool SignalLoggerBase::readDataCollectScript(const std::string & scriptName)
{
  if (!isInitialized_ || isCollectingData_) {
    return false;
  }

  // Lock script
  std::lock_guard<std::mutex> lockScript(scriptMutex_);

  // Check file existance
  struct stat buffer;
  if(stat(scriptName.c_str(), &buffer) == 0)
  {
    // Disable all log data and reallocate buffer
    for(auto & elem : logElements_)
    {
      elem.second->setIsEnabled(false);
    }

    // Load Yaml
    try {
      YAML::Node config = YAML::LoadFile(scriptName);
      for( size_t i = 0; i < config["log_elements"].size(); ++i) {
        std::string name = config["log_elements"][i]["name"].as<std::string>();
        auto elem = logElements_.find(name);
        if(elem != logElements_.end()) {
          elem->second->setIsEnabled(true);
          // Check for divider
          if (YAML::Node parameter = config["log_elements"][i]["divider"]) {
            elem->second->setDivider(parameter.as<int>());
          }
          // Check for action
          if (YAML::Node parameter = config["log_elements"][i]["action"]) {
            elem->second->setAction( static_cast<LogElementInterface::LogElementAction>(parameter.as<int>()) );
          }
          // Check for buffer size
          if (YAML::Node parameter = config["log_elements"][i]["buffer"]["size"]) {
            elem->second->setBufferSize(parameter.as<int>());
          }
          // Check for buffer looping
          if (YAML::Node parameter = config["log_elements"][i]["buffer"]["looping"]) {
            elem->second->setIsBufferLooping(parameter.as<bool>());
          }
        }
        else {
          MELO_WARN_STREAM("Could not load " << name << "from config file. Var not logged.");
        }
      }
    }
    catch(YAML::Exception & e) {
      MELO_WARN_STREAM("Could not load config file, because exception occurred: "<<e.what());
      return false;
    }
  }
  else {
    MELO_ERROR("Logger configuration file can not be opened!");
    return false;
  }

  return true;
}

bool SignalLoggerBase::saveDataCollectScript(const std::string & scriptName)
{
  if (!isInitialized_ || isCollectingData_) {
    return false;
  }

  // Lock script
  std::lock_guard<std::mutex> lockScript(scriptMutex_);

  YAML::Node node;
  std::size_t j = 0;

  for (auto element : logElements_)
  {
    if(element.second->isEnabled()) {
      node["log_elements"][j]["name"] = element.second->getName();
      node["log_elements"][j]["divider"] = element.second->getDivider();
      node["log_elements"][j]["action"] = static_cast<unsigned int>(element.second->getAction());
      node["log_elements"][j]["buffer"]["size"] = element.second->getBufferSize();
      node["log_elements"][j]["buffer"]["looping"] = element.second->isBufferLooping();
      j++;
    }
  }

  // If there are logged elements save them to file
  if(j!=0) {
    std::ofstream outfile(scriptName);
    outfile << node;
    outfile.close();
  }

  return true;

}

}
