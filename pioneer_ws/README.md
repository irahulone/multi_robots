# Pioneer ROS2 System - Enhanced Configuration Management
> **Note**: This documentation was initially auto-generated by AI and has been enhanced with actual system specifications and operational procedures.

## Overview

This enhanced Pioneer ROS2 system provides a comprehensive robotic platform with centralized configuration management, automatic error recovery, and health monitoring capabilities. The system is designed for multiple robot operation with easy deployment and maintenance.

### Key Features

1. **Centralized Configuration**: Single YAML file manages all system settings
2. **Motor Direction Control**: Software-configurable motor polarity from config file
3. **Automatic Error Recovery**: Health monitoring with automatic node restart
4. **Enhanced Sensor Handling**: Robust GPS and IMU integration with retry mechanisms
5. **Multi-Robot Support**: Easy deployment across multiple robots
6. **Systemd Integration**: Automatic startup and service management

## File Structure

```
~/ros2_ws/pioneer_ws/
├── config/
│   └── system_config.yaml          # Central configuration file
├── scripts/
│   ├── setup.sh                    # Enhanced setup script
│   ├── start_ros2.sh              # Auto-generated startup script
│   ├── reset_sensors.py           # Sensor reset utility
│   ├── health_monitor.py          # Health monitoring system
│   └── install.sh                 # Dependency installation
├── convert_pose/                   # GPS/IMU to pose conversion
│   ├── convert_pose/
│   │   └── converter.py           # Enhanced pose converter
│   ├── package.xml
│   └── setup.py
├── gps_core/                       # GPS sensor management
│   ├── gps_core/
│   │   └── run_gps.py             # Enhanced GPS node
│   ├── package.xml
│   └── setup.py
├── imu_core/                       # IMU sensor management
│   ├── imu_core/
│   │   ├── imu_node.py            # Enhanced IMU node
│   │   └── calib_imu.py           # IMU calibration utility
│   ├── package.xml
│   └── setup.py
├── locomotion_core/                # Motor control system
│   ├── locomotion_core/
│   │   ├── movebase_kinematics.py # Configurable motor control
│   │   └── cmd_roboteq.py         # Roboteq motor driver
│   ├── package.xml
│   └── setup.py
└── rover_launch/                   # Launch file configurations
    ├── launch/
    │   ├── rover.launch.py         # Complete system launch
    │   ├── sensor.launch.py        # Sensor-only launch
    │   └── driver.launch.py        # Motor-only launch
    ├── package.xml
    └── setup.py
```

## Prerequisites

- **ROS 2 Jazzy** installed
- **Python 3.8+**
- **Hardware**: Raspberry Pi with GPS and IMU sensors
- **Required Python packages**:
  - `yaml`
  - `psutil`
  - `adafruit-circuitpython-gps`
  - `adafruit-circuitpython-bno055`
- **Sudo access** for systemd service configuration

### Hardware Requirements

- **GPS Module**: Compatible with adafruit_gps library
- **IMU**: BNO055 9-DOF sensor
- **Motor Controller**: Roboteq compatible controller
- **Serial Connections**: USB/UART for GPS and motor controller

## Quick Start

### 1. Installation

```bash
# Navigate to workspace
cd ~/ros2_ws/pioneer_ws

# Install dependencies
./scripts/install.sh

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Configuration

Edit the central configuration file:

```bash
nano ~/ros2_ws/pioneer_ws/config/system_config.yaml
```

**Key configuration sections:**

```yaml
# Robot identification
robot:
  id: "p1"  # Change for each robot (p1, p2, p3, etc.)

# User configuration
user:
  name: "pioneer-i"
  home_dir: "/home/pioneer-i"

# Motor direction configuration (NEW FEATURE)
locomotion:
  max_velocity: 176
  left_motor_sign: -1   # -1 or 1 to invert left motor
  right_motor_sign: 1   # -1 or 1 to invert right motor
```

### 3. System Setup

```bash
cd ~/ros2_ws/pioneer_ws/scripts
./setup.sh
```

This automatically:
- Reads configuration from `system_config.yaml`
- Generates parameter files for each node
- Creates systemd service
- Sets up environment variables
- Builds workspace

### 4. Service Management

```bash
# Check service status
sudo systemctl status ros2_start.service

# View real-time logs
journalctl -u ros2_start.service -f

# Start/stop service manually
sudo systemctl start ros2_start.service
sudo systemctl stop ros2_start.service
```

## Configuration Management

### Motor Direction Control

The new motor direction feature allows software-based control of motor polarity:

```yaml
locomotion:
  left_motor_sign: -1   # Normal: -1, Inverted: 1
  right_motor_sign: 1   # Normal: 1, Inverted: -1
```

**Common configurations:**
- **Standard**: `left_motor_sign: -1, right_motor_sign: 1`
- **Both inverted**: `left_motor_sign: 1, right_motor_sign: -1`
- **Left only inverted**: `left_motor_sign: 1, right_motor_sign: 1`

### GPS Configuration

```yaml
gps:
  device_paths:
    - "/dev/ttyUSB0"
    - "/dev/ttyUSB1"
  baudrate: 9600
  update_rate: 1.0  # Hz
  timer_period: 1.0  # seconds
```

### IMU Configuration

```yaml
imu:
  heading_offset: 0  # degrees
  connection_timeout: 10.0
  calibration_file: "imu_calib/bno055_offsets.json"
```

## Utility Tools

### Sensor Reset Commands

```bash
# Reset both sensors
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor both

# Reset specific sensor
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor gps
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor imu

# Interactive mode
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --interactive

# With retry count
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor both --retries 5
```

### Health Monitoring

```bash
# Start health monitor
~/ros2_ws/pioneer_ws/scripts/health_monitor.py

# Monitor with specific config
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --config ~/ros2_ws/pioneer_ws/config/system_config.yaml

# Verbose logging
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --verbose

# Run as daemon
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --daemon
```

### Health Monitor Features

- **Real-time monitoring**: GPS, IMU, and converter node health
- **Automatic restart**: Failed nodes are automatically restarted
- **Data rate tracking**: Monitors sensor data rates
- **Error counting**: Tracks and manages error thresholds
- **Automatic reset**: Sends reset commands when needed

## Multi-Robot Deployment

### 1. Create Robot-Specific Configuration

```bash
# Copy base configuration
cp ~/ros2_ws/pioneer_ws/config/system_config.yaml ~/robot_configs/p2_config.yaml

# Edit for new robot
nano ~/robot_configs/p2_config.yaml
```

```yaml
robot:
  id: "p2"
user:
  name: "pioneer-ii"
  home_dir: "/home/pioneer-ii"
locomotion:
  left_motor_sign: 1    # Different motor wiring
  right_motor_sign: -1
```

### 2. Deploy to Robot

```bash
# Copy config and deploy
scp ~/robot_configs/p2_config.yaml p2@robot-p2:~/ros2_ws/pioneer_ws/config/system_config.yaml
ssh p2@robot-p2 "cd ~/ros2_ws/pioneer_ws/scripts && ./setup.sh"
```

## Troubleshooting

### Configuration Changes

After modifying `system_config.yaml`:

```bash
# Regenerate parameter files
cd ~/ros2_ws/pioneer_ws/scripts
./setup.sh --regen-params-only

# Restart service
sudo systemctl restart ros2_start.service
```

### Sensor Issues

```bash
# Check sensor connections
ls -la /dev/tty*

# Manual sensor reset
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor both

# Check health monitor logs
journalctl -u ros2_start.service -f | grep -E "(GPS|IMU)"
```

### Node Status Checking

```bash
# List active nodes
ros2 node list | grep p1  # Replace p1 with your robot ID

# Check topic data
ros2 topic echo /p1/gps1
ros2 topic echo /p1/imu/eulerAngle
ros2 topic echo /p1/pose2D

# Monitor health status
ros2 topic echo /p1/health_status
```

### Service Issues

```bash
# Check service status
sudo systemctl status ros2_start.service

# View service configuration
cat /etc/systemd/system/ros2_start.service

# Restart service
sudo systemctl restart ros2_start.service

# View recent logs
journalctl -u ros2_start.service --since "1 hour ago"
```

### Complete System Reset

```bash
# Stop service
sudo systemctl stop ros2_start.service

# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install

# Re-run setup
cd pioneer_ws/scripts
./setup.sh --clean

# Restart service
sudo systemctl start ros2_start.service
```

## ROS Topics

### Published Topics

- `/{robot_id}/gps1` (sensor_msgs/NavSatFix): GPS position data
- `/{robot_id}/imu/quaternion` (geometry_msgs/Quaternion): IMU orientation
- `/{robot_id}/imu/eulerAngle` (std_msgs/Float32MultiArray): Euler angles
- `/{robot_id}/pose2D` (geometry_msgs/Pose2D): Robot pose in local coordinates
- `/{robot_id}/health_status` (std_msgs/Float32MultiArray): System health data

### Subscribed Topics

- `/{robot_id}/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/{robot_id}/reset_gps` (std_msgs/Bool): GPS reset commands
- `/{robot_id}/reset_imu` (std_msgs/Bool): IMU reset commands

### Internal Topics

- `/{robot_id}/ch_vals` (std_msgs/Int32MultiArray): Motor command values

## Performance Monitoring

### Expected Performance

- **GPS Data Rate**: ~1Hz (configurable)
- **IMU Data Rate**: ~10Hz
- **Pose Converter**: ~10Hz
- **CPU Usage**: <50%
- **Memory Usage**: <80%

### Monitoring Commands

```bash
# System resources
htop
free -h
df -h

# ROS performance
ros2 topic hz /p1/gps1
ros2 topic hz /p1/imu/eulerAngle
ros2 topic hz /p1/pose2D

# Network status
ping basestation-ip
```

## Development

### Adding New Features

1. Modify relevant node files
2. Update `system_config.yaml` if new parameters needed
3. Update `setup.sh` to handle new parameters
4. Test with `./setup.sh --regen-params-only`
5. Rebuild workspace: `colcon build --symlink-install`

### Custom Launch Files

Create custom launch files in `rover_launch/launch/`:

```python
# custom_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_id = os.getenv("ROBOT_ID", "p1")
    config_dir = os.path.join(os.getenv("HOME"), "ros2_ws/pioneer_ws/config/nodes")
    
    return LaunchDescription([
        Node(
            package="your_package",
            executable="your_executable",
            parameters=[os.path.join(config_dir, "your_params.yaml")],
            output='screen'
        )
    ])
```

## Advanced Configuration

### Custom Motor Settings

```yaml
locomotion:
  max_velocity: 176      # Normal operation speed
  max_velocity_open: 600 # Open loop maximum
  serial_port: "/dev/ttyACM0"
  baudrate: 115200
  left_motor_sign: -1    # Motor direction control
  right_motor_sign: 1
```

### Health Monitoring Tuning

```yaml
monitoring:
  sensor_timeout: 5.0      # Sensor data timeout (seconds)
  max_node_retries: 3      # Maximum restart attempts
  node_restart_delay: 5.0  # Delay between restarts
```

## Support and Maintenance

### Log Files

- **System logs**: `journalctl -u ros2_start.service`
- **ROS logs**: `~/.ros/log/`
- **Health monitor**: Real-time via health monitor script

### Backup Configuration

```bash
# Backup current configuration
cp ~/ros2_ws/pioneer_ws/config/system_config.yaml ~/backup/system_config_$(date +%Y%m%d).yaml

# Backup generated parameters
tar -czf ~/backup/node_params_$(date +%Y%m%d).tar.gz ~/ros2_ws/pioneer_ws/config/nodes/
```

### Updates and Upgrades

1. **Stop service**: `sudo systemctl stop ros2_start.service`
2. **Backup configuration**: See backup section above
3. **Update code**: Pull latest changes
4. **Rebuild**: `colcon build --symlink-install`
5. **Re-run setup**: `./setup.sh`
6. **Test**: Verify system functionality
7. **Restart service**: `sudo systemctl start ros2_start.service`

This enhanced Pioneer ROS2 system provides a robust, configurable platform for multi-robot operations with comprehensive monitoring and easy maintenance capabilities.