# Required environment

1. Ubuntu 18.04 or 20.04
2. Latest ROS [Melodic](http://wiki.ros.org/melodic/Installation) or [Noetic](http://wiki.ros.org/noetic/Installation)
3. Latest [MAVProxy](https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html#linux)



# Installation
## 1. Install catkin_tool and dependencies
For Melodic:
```
$ sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```  
For Noetic:  
```
$ sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```

## 2. Create the workspace

```
$ mkdir -p ~/mimree_ros/src  
$ cd ~/mimree_ros  
$ catkin init  
$ wstool init src  
```

## 3. Install ROS packages
For MAVLINK, we use the melodic reference **both** Melodic and Noetic as it's not distro-specific and up to date
```
$ rosinstall_generator --rosdistro melodic mavlink | tee src/.rosinstall
```

Next add our custom mavros package reference
```
$ gedit src/.rosinstall
```
Add the following lines at the end of the file.
```
- git:  
    local-name: mavros  
    uri:  https://github.com/ferdianjovan/mavros.git 
    version: lhm-test 
```

Install both packages
```
$ wstool update -t src -j4
$ rosdep install --from-paths src --ignore-src -y
```

Install GeographicLib datasets
```
$ sudo ./src/mavros_mallard/mavros/scripts/
$ install_geographiclib_datasets.sh
```

## 4. Build and source
```
$ catkin build
$ source devel/setup.bash
```

## (Optional) Add source to bashrc
Add source to bashrc to avoid manual source everytime
```
$ echo "source $HOME/devel/setup.bash" >> ~/.bashrc 
```

### Original guides
1. [MAVROS - README](https://github.com/EEEManchester/mavros_mallard/blob/master/README_MAVROS.md)
2. [MAVROS - Installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

# Use

## 1. Setup all the background processes
Open a terminal and run roscore:
```
$ roscore
```

> Test if the installation is successful
> ```
> rosmsg show mavros_msgs/DebugValue
> rossrv show mavros_msgs/ButtonChange
> ```
> Both should return the definition of the rosmsg or rossrv. Otherwise, the installation is not successful.

Open a new terminal, plug in your usb telemetry and find out its port
```
$ ls /dev/ | grep USB
ttyUSB0
```
Run mavproxy, replace `--master=/dev/ttyUSB0` with the correct port.
```
$ mavproxy.py --out=udp:127.0.0.1:14563 --mav20 --console --master=/dev/ttyUSB0
```

Open a third terminal and run mavros.
```
$ roslaunch mavros mimree_apm.launch
```

If everythong goes well, you should see telemetry connected in mavproxy and mavros returns its MAVLINK ID.

## 2. Feedback monitor
To monitor the feedback from LHM, open a fourth terminal and run `rostopic echo`
```
$ rostopic echo /halcyon/mavros/debug_value/debug_vector 
```

## 3. Send command
To send LHM command, open a new terminal and run
```
$ cd mimree_ros/src/mavros/mavros/executables
$ python lhm_executor.py 0
```
Swap `0` with other arguments:
```
help    Displace this message
0       Take off preparation
1       Landing preparation
2       Swing reduction
3       Close hook
4       Open hook
5       Reset controller
```
You can check this message by not providing any arguments or `lhm_executor.py help`