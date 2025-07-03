# uevent_diagnostics_publisher

This ROS package provides a mechanism to publish diagnostic messages based on `uevent` from the Linux kernel. It allows monitoring hardware and system status by interpreting uevent data and providing a standardized way to report health information within a ROS environment.

## Overview

The package receives uevent via a netlink socket. It then uses configurable interpreters (for cameras, sensors, etc.) to extract relevant information.  The extracted data is formatted into ROS diagnostic messages and published on a ROS topic. This enables integration with ROS diagnostic tools for monitoring and alerting.

![uevent_diagnostics_publisher overview](images/uevent_diagnostics_publisher_overview.svg)

## Dependencies

- ROS2 (Humble or later)
- [`diagnostic_updater`](https://index.ros.org/p/diagnostic_updater/) package

## Installation

1.  Clone the package into your ROS workspace:

    ```bash
    mkdir src
    git clone https://github.com/tier4/uevent_diagnostics_publisher.git src/uevent_diagnostics_publisher
    ```

2.  Build the package:

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

3.  Source your workspace:

    ```bash
    source install/setup.bash
    ```

## Usage

```bash
ros2 run uevent_diagnostics_publisher uevent_diagnostics_publisher_node --ros-args -p config_yaml_path:=<CONFIG_YAML_PATH>
```

### Node Parameters

| Parameter Name       | Type     | Default Value | Description                                   |
|:---------------------|:---------|:--------------|:----------------------------------------------|
| `config_yaml_path`   | `string` | `""`          | Path to the YAML configuration file.          |
| `diag_update_period` | `double` | `0.1`         | Period in second that diagnostics are updated |

## YAML Configuration File Format

The `config_yaml_path` parameter points to a YAML file that defines the interpreters and their configurations. The YAML file should iterate the following structure:

```yaml
uevent_description:
  hardware_type:   # the interpreter type for an uevent diag publisher
  devpath:         # sysfs device path
  identifier_key:  # uevent property name
  key_is:          # regular expression that matches the value of identifier_key
  hardware_id:     # which hardware is the uevent associated to?
  value_key:       # uevent property name that tells hardware status
  value_type:      # value type for value_key
  criteria:
    ... # value_type specific criteria description 
```

- **`hardware_type`**: An interpreter type that should handle the uevent.
- **`dev_path`**: A sysfs path in regular expression to which the uevent is associated.
- **`identifier_key`**: A key name to identify which uevent is handled by this interpreter.
- **`key_is`**: A regular expression to match the value of `identifier_key`.
- **`hardware_id`**: A hardware name to be used in output diagnostics message.
- **`value_key`**: The uevent property name that contains a diagnostics name to be monitored.
- **`value_type`**: The type of the status value (e.g., "error_flag"). This determines how the value is interpreted and reported.
- **`criteria`**: The criteria how the value associated with `value_key` will be interpreted.

## Contributing

Feel free to contribute to this package by submitting bug reports, feature requests, or pull requests.
