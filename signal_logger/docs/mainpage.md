# Signal Logger Overview

`signal_logger` provides a configurable logging stack for time-series data.

## Features

Core features:

- configurable ring buffers
- per-signal logging frequencies
- support for primitive, Eigen, and Kindr types
- parallel log-file export
- YAML-based logger scripts

Additional ROS features:

- bag-file export
- topic publishing
- RQT-based logger configuration

## Package Layout

- `signal_logger_core`: shared buffer and logger infrastructure
- `signal_logger_std`: binary and CSV file export
- `signal_logger_ros`: ROS integration
- `signal_logger`: convenience package that combines the available logger types
- `rqt_signal_logger`: RQT GUI for runtime configuration

## Dependencies

The core and standard logger packages are non-ROS and depend primarily on:

- `message_logger`
- `Eigen`
- `kindr`

The ROS integration adds dependencies such as `roscpp` or `rclcpp`,
`std_msgs`, `geometry_msgs`, and `kindr_msgs`.

The RQT plugin additionally depends on `rqt_gui` and `rqt_gui_cpp`.

## Building

Build the main package in a catkin workspace:

```bash
catkin build signal_logger
```

Build the RQT plugin as needed:

```bash
catkin build rqt_signal_logger
```

## Further Documentation

- [Use the signal logger in your program](use_silo.md)
- [Extend the signal logger with additional log types](extend_silo.md)
- [Buffer behavior](../../signal_logger_core/docs/buffer.md)
- [Time handling](../../signal_logger_core/docs/time.md)
- [Logger script format](../../signal_logger_core/docs/log_script.md)
- [Log-file format](../../signal_logger_std/docs/log_file.md)
- [RQT GUI](../../rqt_signal_logger/docs/rqt_gui.md)
