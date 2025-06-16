# Net F/T Sensor Driver

**For MAGICIAN project** Check Installation and Running - Magician version below

This is meta-package that contains ROS2 software for reading the data from F/T sensors
with RDT communication interface such as: ATI F/T sensors, OnRobot F/T sensors.

- `net_ft_driver`: A ros2_control hardware interface for F/T sensor.
- `net_ft_diagnostic_broadcaster`: A ros2_controller for broadcasting a diagnostic
  data from the F/T sensor.
- `net_ft_description`: A package containing F/T sensors' description files.

Currently this package supports only following F/T sensors:

- OnRobot HEX series
- ATI AXIA series
- ATI Net F/T series

Software was tested with `ATI AXIA80` and `OnRobot HEX-E V2` and `ATI Net F/T series`.

## Installation

Installing dependencies:

```Bash
sudo apt update
sudo apt dist-upgrade
rosdep update
git -C src clone --branch humble https://github.com/gbartyzel/ros2_net_ft_driver.git
sudo apt install -y libasio-dev libcurlpp-dev
rosdep install --ignore-src --from-paths src -y -r --rosdistro $ROS_DISTRO
```

Build the package:

```Bash
colcon build 
```

## Running - Magician version
```Bash
ros2 launch net_ft_driver ati_ft.launch.py 
```
Please check this launch for optional arguments to set.

## Running - Original repo

Launch the controller:

```Bash
ros2 launch net_ft_driver net_ft_broadcaster.launch.py ip_address:=192.168.1.1 sensor_type:=ati_axia rdt_sampling_rate:=500
```

where:

- `ip_address`: the IP address of the F/T sensor.
- `sensor_type`: the sensor type, select one of `ati`, `ati_axia`, `onrobot`.
- `rdt_sampling_rate`: the sampling rate of the RDT communication, please refer to
  the sensor manuals for the frequency range.
- `use_hardware_biasing`: whether to use built-in sensor biasing.


## Notes about ATI FT: 
```cpp
/**
Filter Index | Cutoff
-------------|-----------
0            | no filter
1            | 838 Hz
2            | 326 Hz
3            | 152 Hz
4            | 73 Hz
5            | 35 Hz
6            | 18 Hz
7            | 8 Hz
8            | 5 Hz
9            | 1500 Hz
10           | 2000 Hz
11           | 2500 Hz
12           | 3000 Hz
*/
bool AtiFTInterface::set_internal_filter(int value);
```

```cpp
/**
Sets the RDT output rate in Hertz. The actual value used
may be rounded up; see Section 4.7—Communication
Settings Page (comm.htm) for details.
*/
bool AtiFTInterface::set_sampling_rate(int rate)
```
