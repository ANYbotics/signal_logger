# Use the Signal Logger in Your Program

## Access the Logger

The logger is stored in a global shared pointer and can be accessed throughout
the current process.

```cpp
#include <signal_logger/signal_logger.hpp>

signal_logger::logger->...
```

## Select the Logger Type

Include the signal logger header in your application and choose the desired
implementation during initialization.

```cpp
#include <signal_logger/signal_logger.hpp>
```

Available implementations:

- `SignalLoggerNone`

  ```cpp
  signal_logger::setSignalLoggerNone();
  ```

- `SignalLoggerStd`

  ```cpp
  signal_logger::setSignalLoggerStd();
  ```

- `SignalLoggerRos`

  ```cpp
  signal_logger::setSignalLoggerRos(&nh);
  ```

## Initialize the Logger

`SignalLoggerBase::initLogger()` takes `SignalLoggerOptions` such as:

- `updateFrequency_`: frequency of `collectLoggerData()`
- `maxLoggingTime_`: maximum time for fixed-size time buffers
- `collectScriptFileName_`: YAML logger-script path
- `loggerPrefix_`: prefix for logger element names

```cpp
signal_logger::logger->initLogger(options);
```

If `maxLoggingTime_` is zero, the time buffer grows dynamically. Otherwise the
time buffer is preallocated with size `collectFrequency * maxLoggingTime`.

## Add Variables

Use `signal_logger::add()` to register variables:

```cpp
signal_logger::add(
    myVar, name, group, unit, divider, action, bufferType, bufferSize);
```

Key parameters:

- `myVar`: variable to log; its address must stay valid
- `name`: log variable name
- `group`: grouping namespace
- `unit`: physical unit
- `divider`: `collect frequency / log frequency`
- `action`: `signal_logger::LogElementAction`
- `bufferType`: `signal_logger::BufferType`
- `bufferSize`: buffer size

The added variables become active after:

```cpp
signal_logger::logger->updateLogger();
```

## Collect Data

Once the logger is started, call:

```cpp
signal_logger::logger->collectLoggerData();
```

Call it at the configured collection frequency.

## Start and Stop Logging

```cpp
signal_logger::logger->startLogger();
signal_logger::logger->stopLogger();
signal_logger::logger->restartLogger();
```

Starting the logger clears the buffers.

## Save Logged Data

The standard logger can save binary, bag, or CSV output. Saving happens in a
separate thread after the current buffer contents are copied.

```cpp
signal_logger::logger->saveLoggerData(
    {signal_logger::LogFileType::BINARY, signal_logger::LogFileType::CSV});
signal_logger::logger->saveLoggerData(
    {signal_logger::LogFileType::BINARY, signal_logger::LogFileType::CSV},
    "my_custom_log");
signal_logger::logger->stopAndSaveLoggerData();
```

## Publish Logged Data

The ROS logger can publish data in parallel to collection:

```cpp
signal_logger::logger->publishData();
```

Make sure the publishing thread runs frequently enough to avoid data loss.

## Cleanup

Shut the logger down in the destructor of the owning application:

```cpp
signal_logger::logger->cleanup();
```

## Save the Logger Script

Persist the current logger configuration to the default script location with:

```cpp
signal_logger::logger->saveLoggerScript();
```
