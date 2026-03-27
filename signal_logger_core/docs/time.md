# Time Handling

## Collection Timestamps

The signal logger records time automatically on every call to
`collectLoggerData()`. For space efficiency, time is stored once and later
matched to individual log elements when the data is published or saved.

Time is stored as epoch seconds and nanoseconds.

- The standard logger uses the system clock.
- The ROS logger uses `ros::Time::now()` to support simulation time.

## Time Buffer Behavior

### Case 1: `maxLogTime != 0`

A fixed-size buffer of length `maxLogTime * collectFrequency` is allocated. When
it fills up, logging stops automatically.

### Case 2: `maxLogTime == 0`

The time buffer uses `signal_logger::BufferType::EXPONENTIALLY_GROWING` with an
initial capacity of `10 * collectFrequency`, which allows roughly 10 seconds of
recording before the first reallocation.

Growing buffers are expensive to resize, so avoid them in time-critical paths.

### Case 3: All elements use looping buffers

If all active log elements use `signal_logger::BufferType::LOOPING` when the
logger starts, the time buffer also becomes looping. Its size is chosen from
`max(divider * bufferSize)` across the active elements.
