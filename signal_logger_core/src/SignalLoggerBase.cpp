/*!
 * @file     SignalLoggerBase.cpp
 * @author   Gabriel Hottiger, Christian Gehring
 * @date     Sep 28, 2016
 * @brief    Base class for signal loggers.
 */

// signal logger
#include "signal_logger_core/SignalLoggerBase.hpp"

// yaml
#include <yaml-cpp/yaml.h>
#include "signal_logger_core/yaml_helper.hpp"

// boost
#include <boost/iterator/counting_iterator.hpp>

// stl
#include "assert.h"
#include <thread>
#include <fstream>
#include <chrono>
#include <ctime>

// system
#include <sys/stat.h>

namespace signal_logger {

SignalLoggerBase::SignalLoggerBase(const std::string & loggerPrefix):
                                                isInitialized_(false),
                                                isUpdateLocked_(false),
                                                isCollectingData_(false),
                                                isSavingData_(false),
                                                isCopyingBuffer_(false),
                                                isStarting_(false),
                                                noCollectDataCalls_(0),
                                                collectScriptFileName_(LOGGER_DEFAULT_SCRIPT_FILENAME),
                                                updateFrequency_(0),
                                                maxLoggingTime_(0.0),
                                                loggerPrefix_(loggerPrefix),
                                                logElements_(),
                                                logTime_()
{
}

SignalLoggerBase::~SignalLoggerBase()
{

}

void SignalLoggerBase::initLogger(int updateFrequency, const double maxLogTime, const std::string& collectScriptFileName)
{
  // Assert corrupted configuration
  assert(updateFrequency > 0);

  // Set configuration
  updateFrequency_ = updateFrequency;
  maxLoggingTime_ = maxLogTime;
  collectScriptFileName_ = collectScriptFileName;

  // Get time buffer type and logging time
  if(maxLoggingTime_ == 0.0) {
    resetTimeLogElement(signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }
  else {
    resetTimeLogElement(signal_logger::BufferType::FIXED_SIZE, maxLoggingTime_);
  }

  // Notify user
  MELO_INFO("Signal Logger was initialized!");
  isInitialized_ = true;
}

bool SignalLoggerBase::startLogger()
{
  // Warn the user on invalid request
  if(!isInitialized_  || isCollectingData_ || isStarting_)
  {
    MELO_WARN("Signal logger could not be started!%s%s%s", !isInitialized_?" Not initialized!":"",
        isCollectingData_?" Already running!":"", isStarting_?" Delay logger start already requested!":"");
    return false;
  }

  // Still copying the data from the buffer, Wait in other thread until logger can be started.
  if(isCopyingBuffer_) {
    // Save data in different thread
    std::thread t1(&SignalLoggerBase::workerStartLogger, this);
    t1.detach();
    return true;
  }

  // Shared lock on elements for read access
  boost::upgrade_lock<boost::shared_mutex> sharedLockElements(elementsMutex_);

  // If all elements are looping use a looping time buffer
  bool all_looping = enabledElements_.end() == std::find_if(enabledElements_.begin(), enabledElements_.end(),
               [] (const std::pair<std::string, LogElementMapIterator>& s) { return s.second->second->getBufferType() != BufferType::LOOPING; } );

  // Unique lock on time since it is beeing changes
  boost::unique_lock<boost::shared_mutex> uniqueLockTime(timeMutex_);

  if(all_looping) {
    timeElement_->setBufferType(BufferType::LOOPING);
    auto maxElement = std::max_element(enabledElements_.begin(), enabledElements_.end(), maxScaledBufferSize());
    if(maxElement != enabledElements_.end()) {
      timeElement_->setBufferSize(maxElement->second->second->getDivider()*maxElement->second->second->getBufferSize());
      MELO_INFO_STREAM(">>[ SILO: Use Looping Buffer of size:" <<
                       maxElement->second->second->getDivider()*maxElement->second->second->getBufferSize()<<" ]<<");
    }
  }
  else {
    if(maxLoggingTime_ == 0.0) {
      resetTimeLogElement(signal_logger::BufferType::EXPONENTIALLY_GROWING);
    }
    else {
      resetTimeLogElement(signal_logger::BufferType::FIXED_SIZE, maxLoggingTime_);
    }
  }

  // Reset elements
  timeElement_->restartElement();

  boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLockElements(sharedLockElements);
  for(auto & elem : enabledElements_) { elem.second->second->restartElement(); }

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

  boost::unique_lock<boost::shared_mutex> uniqueLockElements(elementsMutex_);

  // Add log elements from temporary list
  {
    boost::unique_lock<boost::shared_mutex> uniqueLockElementsToAdd(elementsToAddMutex_);

    for(auto & elemToAdd : logElementsToAdd_ ) {
      // Transfer ownership to logElements
      logElements_[elemToAdd.first] = std::unique_ptr<LogElementInterface>(std::move(elemToAdd.second));
    }
    // Clear elements
    logElementsToAdd_.clear();
  }

  // Read the script
  if( !readDataCollectScript(collectScriptFileName_) ) {
    MELO_ERROR("Could not load logger script!");
    return false;
  }

  // Store a complete script file to current directory
  if(updateScript) {
    if( !saveDataCollectScript( std::string(LOGGER_DEFAULT_SCRIPT_FILENAME) ) ){
      MELO_ERROR("Could not save logger script!");
      return false;
    }
  }

  return true;

}

void SignalLoggerBase::lockUpdate(bool lock)
{
  isUpdateLocked_ = lock;
}

bool SignalLoggerBase::collectLoggerData()
{
  //! Check if logger is initialized
  if(!isInitialized_) { return false; }

  //! If buffer is copied return -> don't collect
  if(isCopyingBuffer_) { return true; }

  //! Lock for synchronization on saving
  std::unique_lock<std::mutex> collectLock(collectMutex_);

  // Is logger started?
  if(isCollectingData_)
  {
    // Set current time
    logTime_ = this->getCurrentTime();

    // Check if time buffer is full when using a fixed size buffer
    if(timeElement_->getBufferType() == BufferType::FIXED_SIZE && timeElement_->noItemsInBuffer() == timeElement_->getBufferSize())
    {
      MELO_WARN("Logger stopped. Time buffer is full!");
      this->stopLogger();
      return true;
    }

    // Lock elements
    boost::shared_lock<boost::shared_mutex> sharedLockElements(elementsMutex_);
    boost::shared_lock<boost::shared_mutex> sharedLockTime(timeMutex_);

    // Lock all mutexes
    for(auto & elem : enabledElements_)
    {
      elem.second->second->acquireMutex().lock();
    }

    // Update time
    {
      std::unique_lock<std::mutex> timeLock(timeElement_->acquireMutex());
      timeElement_->collectData();
    }

    // Collect element into buffer
    for(auto & elem : enabledElements_)
    {
      if((noCollectDataCalls_ % elem.second->second->getDivider()) == 0) {
        elem.second->second->collectData();
      }
      elem.second->second->acquireMutex().unlock();
    }

    ++noCollectDataCalls_;
  }

  return true;
}

bool SignalLoggerBase::publishData()
{
  // Try to lock elementsMutex
  boost::shared_lock<boost::shared_mutex> lock(elementsMutex_, boost::try_to_lock);
  if(lock) {
    boost::shared_lock<boost::shared_mutex> lockTime(timeMutex_, boost::try_to_lock);
    if(lockTime) {
      // Publish data from buffer
      for(auto & elem : enabledElements_)
      {

        if(elem.second->second->isPublished())
        {
          elem.second->second->publishData(*timeElement_, noCollectDataCalls_);
        }
      }
    }
  }

  return true;
}

bool SignalLoggerBase::saveLoggerData(LogFileType logfileType)
{
  if(!isInitialized_)
  {
    MELO_WARN("Signal logger could not save data! Not initialized!");
    return false;
  }

  // Save data in different thread
  std::thread t1(&SignalLoggerBase::workerSaveDataWrapper, this, logfileType);
  t1.detach();

  return true;
}

bool SignalLoggerBase::stopAndSaveLoggerData()
{
  return this->stopLogger() && this->saveLoggerData();
}

bool SignalLoggerBase::cleanup()
{
  boost::unique_lock<boost::shared_mutex> lock(elementsMutex_);

  // Publish data from buffer
  for(auto & elem : enabledElements_)
  {
    elem.second->second->cleanupElement();
  }
  return true;
}


bool SignalLoggerBase::readDataCollectScript(const std::string & scriptName)
{
  if (!isInitialized_ || isCollectingData_) {
    MELO_WARN("Not initialized or collecting!");
    return false;
  }

  // Check filename size
  std::string ending = ".yaml";
  if ( (ending.size() + 1) > scriptName.size()) {
    MELO_ERROR_STREAM("Scriptname must be a yaml file : *.yaml");
    return false;
  }
  // Check file ending
  if(!std::equal(ending.rbegin(), ending.rend(), scriptName.rbegin())) {
    MELO_ERROR_STREAM("Scriptname must be a yaml file : *.yaml");
    return false;
  }

  // Lock script
  std::lock_guard<std::mutex> lockScript(scriptMutex_);

  // Check file existance
  struct stat buffer;
  if(stat(scriptName.c_str(), &buffer) != 0) {
    std::fstream fs;
    fs.open(scriptName, std::fstream::out);
    fs.close();
  }
  // Disable all log data and reallocate buffer
  for(auto & elem : enabledElements_) { elem.second->second->setIsEnabled(false); }
  enabledElements_.clear();

  // Get set of numbers from 0...(size-1)
  std::set<unsigned int> iteratorOffsets(boost::counting_iterator<unsigned int>(0),
                                         boost::counting_iterator<unsigned int>(logElements_.size()));

  // Save loading of yaml config file
  try {
    YAML::Node config = YAML::LoadFile(scriptName);
    if (config["log_elements"].IsSequence())
    {
      YAML::Node logElementsNode = config["log_elements"];
      for( size_t i = 0; i < logElementsNode.size(); ++i)
      {
        if (YAML::Node parameter = logElementsNode[i]["name"])
        {
          std::string name = logElementsNode[i]["name"].as<std::string>();
          auto elem = logElements_.find(name);
          if(elem != logElements_.end())
          {
            // Erase the iterator offset that belongs to elem since it was found in the yaml
            iteratorOffsets.erase( std::distance(logElements_.begin(), elem) );

            // Overwrite defaults if specified in yaml file
            if (YAML::Node parameter = logElementsNode[i]["enabled"])
            {
              elem->second->setIsEnabled(parameter.as<bool>());
            }
            else {
              // Disable element if nothing is specified
              elem->second->setIsEnabled(false);
            }
            // Overwrite defaults if specified in yaml file
            if (YAML::Node parameter = logElementsNode[i]["divider"])
            {
              elem->second->setDivider(parameter.as<int>());
            }
            // Check for action
            if (YAML::Node parameter = logElementsNode[i]["action"])
            {
              elem->second->setAction( static_cast<LogElementAction>(parameter.as<int>()) );
            }
            // Check for buffer size
            if (YAML::Node parameter = logElementsNode[i]["buffer"]["size"])
            {
              elem->second->setBufferSize(parameter.as<int>());
            }
            // Check for buffer looping
            if (YAML::Node parameter = logElementsNode[i]["buffer"]["type"])
            {
              elem->second->setBufferType( static_cast<BufferType>(parameter.as<int>()) );
            }
            // Insert element
            if(elem->second->isEnabled()) {
              enabledElements_.insert(std::pair<std::string, LogElementMapIterator>(name, elem));
            }
          }
          else {
            MELO_DEBUG_STREAM("Could not load " << name << "from config file. Var not logged.");
          }
        }
        else {
          MELO_DEBUG_STREAM("Could not load get name from config file. Ignore entry nr: ." << i << "!");
        }
      }
    }
    else {
      MELO_DEBUG_STREAM("Parameter file is ill-formatted. Log elements is no sequence.");
    }
  }
  catch(YAML::Exception & e) {
    MELO_ERROR_STREAM("Could not load config file, because exception occurred: "<<e.what());
    return false;
  }

  // Noisily add all elements that were not in the configuration file
  for(auto iteratorOffset : iteratorOffsets) {
    LogElementMapIterator elem = std::next(logElements_.begin(), iteratorOffset);
    elem->second->setIsEnabled(true);
    enabledElements_.insert(std::pair<std::string, LogElementMapIterator>(elem->first, elem));
    MELO_DEBUG("Enable logger element %s. It was not specified in the logger file!", elem->first.c_str() );
  }

  return true;
}

bool SignalLoggerBase::saveDataCollectScript(const std::string & scriptName)
{
  if (!isInitialized_ || isCollectingData_) {
    return false;
  }

  // Check filename size
  std::string ending = ".yaml";
  if ( (ending.size() + 1) > scriptName.size()) {
    MELO_ERROR_STREAM("[Signal Logger] Scriptname must be a yaml file : *.yaml");
    return false;
  }
  // Check file ending
  if(!std::equal(ending.rbegin(), ending.rend(), scriptName.rbegin())) {
    MELO_ERROR_STREAM("[Signal Logger] Scriptname must be a yaml file : *.yaml");
    return false;
  }

  // Lock script
  std::lock_guard<std::mutex> lockScript(scriptMutex_);

  // Push back all data to node
  YAML::Node node;
  std::size_t j = 0;

  for (auto & elem : logElements_)
  {
    node["log_elements"][j]["name"] = elem.second->getName();
    node["log_elements"][j]["enabled"] = elem.second->isEnabled();
    node["log_elements"][j]["divider"] = elem.second->getDivider();
    node["log_elements"][j]["action"] = static_cast<int>(elem.second->getAction());
    node["log_elements"][j]["buffer"]["size"] = elem.second->getBufferSize();
    node["log_elements"][j]["buffer"]["type"] = static_cast<int>(elem.second->getBufferType());
    j++;
  }

  // If there are logged elements save them to file
  if(j!=0) {
    std::ofstream outfile(scriptName);
    yaml_helper::writeYamlOrderedMaps(outfile, node);
    outfile.close();
  }

  return j!=0;

}

bool SignalLoggerBase::resetTimeLogElement(signal_logger::BufferType buffertype, double maxLogTime) {

  timeElement_.reset(new LogElementBase<TimestampPair>(&logTime_, loggerPrefix_ + std::string{"/time"}, "[s/ns]", 1,
                                                       LogElementAction::SAVE, maxLogTime*updateFrequency_, buffertype));
  return true;
}

signal_logger::TimestampPair SignalLoggerBase::getCurrentTime() {
  signal_logger::TimestampPair timeStamp;

  // Get time in seconds and nanoseconds
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  timeStamp.first = seconds.count();
  timeStamp.second = std::chrono::duration_cast<std::chrono::nanoseconds>(duration-seconds).count();

  return timeStamp;
}

bool SignalLoggerBase::workerSaveDataWrapper(LogFileType logfileType) {
  // Check if data already saved
  if(isSavingData_) {
    MELO_WARN("Is already saving data!");
    return false;
  }

  // set save flag
  isSavingData_ = true;

  // Read suffix number from file
  int suffixNumber = 0;
  std::ifstream ifs(".last_data", std::ifstream::in);
  if(ifs.is_open()) { ifs >> suffixNumber; }
  ifs.close();
  ++suffixNumber;

  // Write next suffix number to file
  std::ofstream ofs(".last_data", std::ofstream::out | std::ofstream::trunc);
  if(ofs.is_open()) { ofs << suffixNumber; }
  ofs.close();

  // To string with format 00000 (e.g 00223)
  std::string suffixString = std::to_string(suffixNumber);
  while(suffixString.length() != 5 ) { suffixString.insert(0, "0"); }

  // Get local time
  char dateTime[21];
  std::time_t now = std::chrono::system_clock::to_time_t ( std::chrono::system_clock::now() );
  strftime(dateTime, sizeof dateTime, "%Y%m%d_%H-%M-%S_", std::localtime(&now));

  // Filename format (e.g. silo_13Sep2016_12-13-49_00011)
  std::string filename = std::string{"silo_"} + std::string{dateTime} + suffixString;

  // Set flag
  isCopyingBuffer_ = true;

  // Wait for collecting to be done
  // Note: is collecting data remains untouched
  {
    std::unique_lock<std::mutex> collectLock(collectMutex_);
  }

  // Copy general stuff
  noCollectDataCallsCopy_ = noCollectDataCalls_.load();

  {
    boost::upgrade_lock<boost::shared_mutex> upgradeElementlock(elementsMutex_);

    // Copy data from buffer
    for(auto & elem : enabledElements_)
    {
      if(elem.second->second->isSaved())
      {
        elem.second->second->createLocalBufferCopy();
      }
    }

    boost::upgrade_lock<boost::shared_mutex> upgradeTimeLock(timeMutex_);
    timeElement_->createLocalBufferCopy();

    // Reset elements
    {
      // Wait for publish to complete
      boost::upgrade_to_unique_lock<boost::shared_mutex> elementLock(upgradeElementlock);
      boost::upgrade_to_unique_lock<boost::shared_mutex> timeLock(upgradeTimeLock);

      // Reset buffers and counters
      noCollectDataCalls_ = 0;

      // Clear buffer for log elements
      for(auto & elem : enabledElements_) {
        elem.second->second->restartElement();
      }

      // Clear buffer for time elements
      timeElement_->restartElement();

      // Set flag -> collection can restart
      isCopyingBuffer_ = false;
    }

  }

  // Start saving copy to file
  bool success = this->workerSaveData(filename, logfileType);

  // Set flag, notify user
  isSavingData_ = false;

  MELO_INFO_STREAM( "All done, captain! Stored logging data to file " << filename );

  return success;
}

bool SignalLoggerBase::workerStartLogger() {
  isStarting_ = true;

  while(true) {
    if(!isSavingData_) {
      isStarting_ = false;
      MELO_INFO("Delayed logger start!");
      return startLogger();
    }
    // Sleep for one timestep
    usleep( (1.0/updateFrequency_) * 1e6 );
  }
}


}
