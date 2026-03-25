# Extend the Signal Logger With Additional Log Types

## Add a New Data Type

To add a new type, provide the traits needed to publish and serialize that type.
For built-in support, update:

- `signal_logger_ros_traits.hpp`
- `signal_logger_std_traits.hpp`
- `LogElementTypes.hpp`

The buffer is templated on the logged type. If the default buffer handling is
not sufficient for the new type, extend the corresponding customization in
`signal_logger_core/Buffer.hpp`.

## Example: Add a `Circle` Type

### 1. Add the Type Definition

```cpp
struct Circle {
  double diameter;
  Eigen::Vector2d center;
};
```

### 2. Add Standard-Logger Traits

The standard logger traits are designed so they can be reused by wrapper types.
Each helper receives an accessor that returns the logged value from the owning
container.

```cpp
template<typename ContainerType_>
struct sls_traits<signal_logger::Circle, ContainerType_> {
  static void writeLogElementToStreams(
      std::stringstream* text,
      std::stringstream* binary,
      signal_logger::LogFileType fileType,
      const signal_logger::Buffer<ContainerType_>& buffer,
      const std::string& name,
      const std::size_t divider,
      const unsigned int startDiff,
      const unsigned int endDiff,
      const std::function<const signal_logger::Circle*(
          const ContainerType_ * const)>& accessor =
          [](const ContainerType_ * const v) { return v; }) {
    auto getDiameter = [accessor](const ContainerType_ * const v) {
      return &(accessor(v)->diameter);
    };
    sls_traits<double, ContainerType_>::writeLogElementToStreams(
        text, binary, fileType, buffer, name + "_diameter", divider,
        startDiff, endDiff, getDiameter);

    auto getCenter = [accessor](const ContainerType_ * const v) {
      return &(accessor(v)->center);
    };
    sls_traits<Eigen::Vector2d, ContainerType_>::writeLogElementToStreams(
        text, binary, fileType, buffer, name + "_center", divider,
        startDiff, endDiff, getCenter);
  }
};
```

### 3. Add ROS Traits

If the ROS logger should publish the new type, add a message to
`signal_logger_msgs` and provide the ROS traits:

```text
# CircleStamped.msg
Header header
float64 diameter
float64 center_x
float64 center_y
```

```cpp
template<>
struct slr_msg_traits<signal_logger::Circle> {
  using msgtype = signal_logger_msgs::CircleStamped;
};

template<>
struct slr_update_traits<signal_logger::Circle> {
  static void updateMsg(const signal_logger::Circle* var,
                        typename slr_msg_traits<signal_logger::Circle>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->diameter = var->diameter;
    msg->center_x = var->center(0);
    msg->center_y = var->center(1);
  }
};
```

## Keep Project-Specific Types Local

If a type would introduce an unnecessary dependency into the shared
`signal_logger` packages, keep its traits inside your own project instead of
upstreaming them.

```cpp
namespace signal_logger_std {
namespace traits {
// Standard traits for the project-specific type.
}
}  // namespace signal_logger_std

namespace signal_logger_ros {
namespace traits {
// ROS traits for the project-specific type.
}
}  // namespace signal_logger_ros
```

After including that header together with `signal_logger.hpp`, the custom type
can be added to the logger like any built-in one.
