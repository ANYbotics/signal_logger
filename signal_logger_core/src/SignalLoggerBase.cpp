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

SignalLoggerBase::SignalLoggerBase():
                  options_(),
                  noCollectDataCalls_(0u),
                  noCollectDataCallsCopy_(0u),
                  logElements_(),
                  enabledElements_(),
                  logElementsToAdd_(),
                  logTime_(),
                  timeElement_(),
                  isInitialized_(false),
                  isCollectingData_(false),
                  isSavingData_(false),
                  isCopyingBuffer_(false),
                  isStarting_(false),
                  shouldPublish_(false),
                  loggerMutex_(),
                  saveLoggerDataMutex_()
{
}

SignalLoggerBase::~SignalLoggerBase()
{

}

void SignalLoggerBase::initLogger(const SignalLoggerOptions& options)
{
  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> tryInitLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryInitLoggerLock && isSavingData_) {
    MELO_WARN("Saving data while trying to initialize. Do nothing!");
    return;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> initLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryInitLoggerLock) { initLoggerLock.lock(); }

  // Assert corrupted configuration
  assert(options.updateFrequency_ > 0);

  // Set configuration
  options_ = options;

  // Get time buffer type and logging time
  if(options_.maxLoggingTime_ == 0.0) {
    resetTimeLogElement(BufferType::EXPONENTIALLY_GROWING, LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME);
  }
  else {
    resetTimeLogElement(BufferType::FIXED_SIZE, options_.maxLoggingTime_);
  }

  // Notify user
  MELO_INFO("[Signal Logger] Initialized!");
  isInitialized_ = true;

}

bool SignalLoggerBase::startLogger()
{

  // Delayed logger start if saving prevents lock of mutex
  boost::unique_lock<boost::shared_mutex> tryStartLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryStartLoggerLock && isCopyingBuffer_) {
    // Still copying the data from the buffer, Wait in other thread until logger can be started.
    std::thread t1(&SignalLoggerBase::workerStartLogger, this);
    t1.detach();
    return true;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> startLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryStartLoggerLock) { startLoggerLock.lock(); }

  // Warn the user on invalid request
  if(!isInitialized_  || isCollectingData_ || isStarting_)
  {
    MELO_WARN("[Signal logger] Could not start!%s%s%s", !isInitialized_?" Not initialized!":"",
        isCollectingData_?" Already running!":"", isStarting_?" Delayed logger start was already requested!":"");
    return false;
  }

  if(isCopyingBuffer_) {
    // Still copying the data from the buffer, Wait in other thread until logger can be started.
    std::thread t1(&SignalLoggerBase::workerStartLogger, this);
    t1.detach();
    return true;
  }

  // If all elements are looping use a looping time buffer
  auto loopingElement = std::find_if(enabledElements_.begin(), enabledElements_.end(),
                                     [] (const LogElementMapIterator& s) { return s->second->getBuffer().getBufferType() != BufferType::LOOPING; } );

  if( loopingElement == enabledElements_.end() ) {
    unsigned int timeBufferSize = 0;
    auto maxElement = std::max_element(enabledElements_.begin(), enabledElements_.end(), maxScaledBufferSize());
    if(maxElement != enabledElements_.end()) {
      timeBufferSize = (*maxElement)->second->getOptions().getDivider() * (*maxElement)->second->getBuffer().getBufferSize();
    }
    timeElement_->getBuffer().setBufferType(BufferType::LOOPING);
    timeElement_->getBuffer().setBufferSize((timeBufferSize);
    MELO_INFO_STREAM("[Signal logger] Use Looping Buffer of size:" << timeBufferSize);
  }
  else {
    if(options_.maxLoggingTime_ == 0.0) {
      timeElement_->getBuffer().setBufferType(signal_logger::BufferType::EXPONENTIALLY_GROWING);
      timeElement_->getBuffer().setBufferSize(LOGGER_EXP_GROWING_MAXIMUM_LOG_TIME*options_.updateFrequency_);
    }
    else {
      timeElement_->getBuffer().setBufferType(signal_logger::BufferType::FIXED_SIZE);
      timeElement_->getBuffer().setBufferSize(options_.maxLoggingTime_*options_.updateFrequency_);
    }
  }

  // Reset elements
  timeElement_->reset();


  for(auto & elem : enabledElements_) { elem->second->reset(); }

  // Reset flags and data collection calls
  isCollectingData_ = true;
  shouldPublish_ = true;
  noCollectDataCalls_ = 0;

  return true;
}

bool SignalLoggerBase::stopLogger()
{
  // If publishData is running notify stop
  shouldPublish_ = false;

  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> tryStopLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryStopLoggerLock && isSavingData_) {
    MELO_WARN("Saving data while trying to stop logger. Do nothing!");
    return false;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> stopLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryStopLoggerLock) { stopLoggerLock.lock(); }

  if(!isInitialized_ || !isCollectingData_)
  {
    MELO_WARN("[Signal logger] Could not stop!%s%s", !isInitialized_?" Not initialized!":"", !isCollectingData_?" Not running!":"");
    return false;
  }

  isCollectingData_ = false;

  return true;
}

bool SignalLoggerBase::restartLogger()
{
  // Mutex is locked internally
  bool stopped = stopLogger();
  return startLogger() && stopped;
}

bool SignalLoggerBase::updateLogger() {

  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> tryUpdateLoggerLock(loggerMutex_, boost::try_to_lock);
  if(!tryUpdateLoggerLock && isSavingData_) {
    MELO_WARN("Saving data while trying to update logger. Do nothing!");
    return false;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> updateLoggerLock(loggerMutex_, boost::defer_lock);
  if(!tryUpdateLoggerLock) { updateLoggerLock.lock(); }

  // Check if update logger call is valid
  if(!isInitialized_ || isCollectingData_ || isSavingData_)
  {
    MELO_WARN("[Signal logger] Could not update!%s%s%s", !isInitialized_?" Not initialized!":"",
        isSavingData_?" Saving data!":"", isCollectingData_?" Collecting data!":"");
    return false;
  }

  // Add elements to list
  for(auto & elemToAdd : logElementsToAdd_ ) {
    // Transfer ownership to logElements
    logElements_[elemToAdd.first] = std::unique_ptr<LogElementInterface>(std::move(elemToAdd.second));
  }
  // Clear elements
  logElementsToAdd_.clear();

  // Read the script
  if( !readDataCollectScript(options_.collectScriptFileName_) ) {
    MELO_ERROR("[Signal logger] Could not load logger script!");
    return false;
  }

  return true;

}

bool SignalLoggerBase::saveLoggerScript() {
  // If lock can not be acquired because of saving ignore the call
  boost::unique_lock<boost::shared_mutex> trySaveLoggerScriptLock(loggerMutex_, boost::try_to_lock);
  if(!trySaveLoggerScriptLock && isSavingData_) {
    MELO_WARN("Saving data while trying to save logger script. Do nothing!");
    return false;
  }

  // Lock the logger if not locked yet (blocking!)
  boost::unique_lock<boost::shared_mutex> saveLoggerScriptLock(loggerMutex_, boost::defer_lock);
  if(!trySaveLoggerScriptLock) { saveLoggerScriptLock.lock(); }

  if( !saveDataCollectScript( std::string(LOGGER_DEFAULT_SCRIPT_FILENAME) ) ){
    MELO_ERROR("[Signal logger] Could not save logger script!");
    return false;
  }
  return true;
}

bool SignalLoggerBase::collectLoggerData()
{
  // Try lock logger for read (non blocking!)
  boost::shared_lock<boost::shared_mutex> collectLoggerDataLock(loggerMutex_, boost::try_to_lock);

  if(collectLoggerDataLock)
  {
    //! Check if logger is initialized
    if(!isInitialized_) { return false; }

    //! If buffer is copied return -> don't collect
    if(isCopyingBuffer_) { return true; }

    // Is logger started?
    if(isCollectingData_)
    {
      // Set current time (thread safe, since only used in init, start which have unique locks on the logger)
      logTime_ = this->getCurrentTime();

      // Check if time buffer is full when using a fixed size buffer
      if(timeElement_->getBuffer().getBufferType() == BufferType::FIXED_SIZE && timeElement_->getBuffer().noTotalItems() == timeElement_->getBuffer().getBufferSize())
      {
        MELO_WARN("[Signal Logger] Stopped. Time buffer is full!");
        // Free lock and stop logger
        collectLoggerDataLock.unlock();
        this->stopLogger();
        return true;
      }

      // Lock all mutexes for time synchronization
      for(auto & elem : enabledElements_)
      {
        elem->second->acquireMutex().lock();
      }

      // Update time
      {
        std::unique_lock<std::mutex> uniqueTimeElementLock(timeElement_->acquireMutex());
        timeElement_->collectData();
      }

      // Collect element into buffer and unlock element mutexes (time/value pair is valid now)
      for(auto & elem : enabledElements_)
      {
        if((noCollectDataCalls_ % elem->second->getOptions().getDivider()) == 0) {
          elem->second->collectData();
        }
        elem->second->acquireMutex().unlock();
      }

      // Iterate
      ++noCollectDataCalls_;
    }
  }
  else {
    MELO_DEBUG("[Signal Logger] Dropping collect data call!");
  }

  return true;
}

bool SignalLoggerBase::publishData()
{
  // Check if publishing is desired
  if(!shouldPublish_) {
    return true;
  }

  // Try lock logger for read (non blocking!)
  boost::shared_lock<boost::shared_mutex> publishDataLock(loggerMutex_, boost::try_to_lock);

  if(publishDataLock)
  {
    // Publish data from buffer
    for(auto & elem : enabledElements_)
    {
      // Publishing blocks the mutex for quite a long time, allow interference by stopLogger using atomic bool
      if(shouldPublish_) {
        if(elem->second->getOptions().isPublished()) {
          elem->second->publishData(*timeElement_, noCollectDataCalls_);
        }
      }
      else {
        return true;
      }
    }
  }

  return true;
}

bool SignalLoggerBase::saveLoggerData(LogFileType logfileType)
{
  // Make sure start stop are not called in the meantime
  boost::shared_lock<boost::shared_mutex> saveLoggerDataLock(loggerMutex_);

  // Allow only single call to save logger data
  std::unique_lock<std::mutex> lockSaveData(saveLoggerDataMutex_, std::try_to_lock);

  if(lockSaveData) {
    if(!isInitialized_ || isSavingData_)
    {
      MELO_WARN("[Signal logger] Could not save data! %s%s", isSavingData_?" Already saving data!":"",
          !isInitialized_?" Not initialized!":"");
      return false;
    }

    // set save flag
    isCopyingBuffer_ = true;
    isSavingData_ = true;

    // Save data in different thread
    std::thread t1(&SignalLoggerBase::workerSaveDataWrapper, this, logfileType);
    t1.detach();
  }
  else {
    MELO_WARN("[Signal logger] Already saving to file!");
  }

  return true;
}

bool SignalLoggerBase::stopAndSaveLoggerData()
{
  // Mutex is locked internally
  bool stopped = stopLogger();
  return saveLoggerData() && stopped;
}

bool SignalLoggerBase::cleanup()
{
  // Lock the logger (blocking!)
  boost::unique_lock<boost::shared_mutex> updateLoggerLock(loggerMutex_);

  // Publish data from buffer
  for(auto & elem : logElements_) { elem.second->cleanup(); }
  for(auto & elem : logElementsToAdd_) { elem.second->cleanup(); }

  // Clear maps
  enabledElements_.clear();
  logElements_.clear();
  logElementsToAdd_.clear();

  return true;
}

bool SignalLoggerBase::readDataCollectScript(const std::string & scriptName)
{
  if(!isInitialized_ || isSavingData_)
  {
    MELO_WARN("[Signal logger] Could not read data collect script! %s%s", isCollectingData_?" Collecting data!":"",
        !isInitialized_?" Not initialized!":"");
    return false;
  }

  // Check filename size and ending
  std::string ending = ".yaml";
  if ( ( (ending.size() + 1) > scriptName.size() ) || !std::equal(ending.rbegin(), ending.rend(), scriptName.rbegin()) ) {
    MELO_ERROR_STREAM("[Signal logger] Script must be a yaml file : *.yaml");
    return false;
  }

  // Check file existance
  struct stat buffer;
  if(stat(scriptName.c_str(), &buffer) != 0) {
    std::fstream fs;
    fs.open(scriptName, std::fstream::out);
    fs.close();
  }

  // Disable all log data and reallocate buffer
  // Thread-safe since these methods are only called by update logger which owns a unique lock of the elements map
  for(auto & elem : enabledElements_) { elem->second->setIsEnabled(false); }
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
              enabledElements_.push_back(elem);
            }
          }
          else {
            MELO_DEBUG_STREAM("[Signal logger] Could not load " << name << " from config file. Var not logged.");
          }
        }
        else {
          MELO_DEBUG_STREAM("[Signal logger] Could not load get name from config file. Ignore entry nr: " << i << "!");
        }
      }
    }
    else {
      MELO_DEBUG_STREAM("[Signal logger] Parameter file is ill-formatted. Log elements is no sequence.");
    }
  }
  catch(YAML::Exception & e) {
    MELO_ERROR_STREAM("[Signal logger] Could not load config file, because exception occurred: "<< e.what());
    return false;
  }

  // Noisily add all elements that were not in the configuration file
  for(auto iteratorOffset : iteratorOffsets) {
    LogElementMapIterator elem = std::next(logElements_.begin(), iteratorOffset);
    elem->second->setIsEnabled(true);
    enabledElements_.push_back(elem);
    MELO_DEBUG("[Signal logger] Enable logger element %s. It was not specified in the logger file!", elem->first.c_str() );
  }

  return true;
}

bool SignalLoggerBase::saveDataCollectScript(const std::string & scriptName)
{
  if(!isInitialized_ || isSavingData_)
  {
    MELO_WARN("[Signal logger] Could not save data collect script! %s%s", isCollectingData_?" Collecting data!":"",
        !isInitialized_?" Not initialized!":"");
    return false;
  }

  // Check filename size and ending
  std::string ending = ".yaml";
  if ( ( (ending.size() + 1) > scriptName.size() ) || !std::equal(ending.rbegin(), ending.rend(), scriptName.rbegin()) ) {
    MELO_ERROR_STREAM("[Signal logger] Script must be a yaml file : *.yaml");
    return false;
  }

  // Push back all data to node
  YAML::Node node;
  std::size_t j = 0;

  // Update logger owns a unique lock for the log element map -> thread-safe
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
  if(j != 0) {
    std::ofstream outfile(scriptName);
    yaml_helper::writeYamlOrderedMaps(outfile, node);
    outfile.close();
  }

  return (j != 0);
}

bool SignalLoggerBase::resetTimeLogElement(signal_logger::BufferType buffertype, double maxLogTime) {
  // Reset time element
  timeElement_.reset(new LogElementBase<TimestampPair>(&logTime_, options_.loggerPrefix_ + std::string{"/time"}, "[s/ns]", 1,
                                                       LogElementAction::SAVE, maxLogTime*options_.updateFrequency_, buffertype));
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
  //-- Produce file name, this can be done in series to every other process

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

  {
    // Lock the logger (blocking!)
    boost::unique_lock<boost::shared_mutex> workerSaveDataWrapperLock(loggerMutex_);

    // Copy general stuff
    noCollectDataCallsCopy_ = noCollectDataCalls_.load();

    // Copy data from buffer
    for(auto & elem : enabledElements_)
    {
      if(elem->second->isSaved())
      {
        elem->second->createLocalBufferCopy();
      }
    }

    // Copy time
    timeElement_->createLocalBufferCopy();

    // Reset buffers and counters
    noCollectDataCalls_ = 0;

    // Clear buffer for log elements
    for(auto & elem : enabledElements_) {
      elem->second->restartElement();
    }

    // Clear buffer for time elements
    timeElement_->restartElement();

    // Set flag -> collection can restart
    isCopyingBuffer_ = false;
  }

  // Start saving copy to file
  bool success = this->workerSaveData(filename, logfileType);

  // Set flag, notify user
  isSavingData_ = false;

  MELO_INFO_STREAM( "[Signal logger] All done, captain! Stored logging data to file " << filename );

  return success;
}

bool SignalLoggerBase::workerStartLogger() {
  isStarting_ = true;

  while(true) {
    if(!isCopyingBuffer_) {
      isStarting_ = false;
      MELO_INFO("[Signal logger] Delayed logger start!");
      return startLogger();
    }
    // Sleep for one timestep (init is only allowed once. access of updateFrequency is ok)
    usleep( (1.0/options_.updateFrequency_) * 1e6 );
  }
}


}
