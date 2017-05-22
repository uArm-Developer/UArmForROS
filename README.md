# UArmForROS
This is the uarm ROS package designed by Joey Song ( joey@ufactory.cc / astainsong@gmail.com)

## 0. Quickstart and Execution
---
Three ways to control uArm movements
![](http://obmqyor62.bkt.clouddn.com/ROS_QuickStart.jpg)

## 1. Installation
---
### 1.1 Pre-Requirements
Connect uArm and get USB permission to access uArm
```bash
$ cd /etc/udev/rules.d
```
Creat a file `ttyUSB.rules` and put the following line: `KERNEL=="ttyUSB*", MODE="0666"`. Save the file and **reconnect** uArm USB to make it effective. (if you already have the permission to access USB, you can skip this step)
> If your uArm is connected to ttyACM instead of ttyUSB (you can check using dmesg -c command after connecting your uArm),
Creat a file `ttyACM.rules` and put the following line: `KERNEL=="ttyACM*", MODE="0666"`. Save the file and **reconnect** uArm USB to make it effective. (if you already have the permission to access USB, you can skip this step)


For using this package, the [pyUarm](https://github.com/uArm-Developer/pyuarm) library **SHOULD** be installed at first.

```bash
$ pip install pyuarm
```

And check your uArms's VID:PID using lsusb after connecting uArm.
```bash
$ lsusb
Bus 001 Device 020: ID 1234:1234 Arduino SA Mega 2560 R3 (CDC ACM)
```

And you need to specify your device number by editing this file
e.g. if you prefer nano as an editor)
```bash
$ sudo nano /usr/local/lib/python2.7/dist-packages/pyuarm/tools/list_uarms.py
```
In this file, you need to change this line, VID:PID with your own device.

```python
UARM_HWID_KEYWORD = "USB VID:PID=1234:1234"
```


Connect uArm to computer and upgrade your uArmProtocol Firmware
```bash
$ uarm-firmware -u
```
or
```bash
$ python -m pyuarm.tools.firmware -d
```

### 1.2 Package Download and Install
Install ros package in your src folder of your [Catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/uArm-Developer/UArmForROS.git
$ cd ~/catkin_ws
$ catkin_make
```
## 2. Source FIles
---
Before you use any packages in uarmForROS, source all setup.bash files which allow you to access uarm package
 ```bash

# System configure ROS environment variables automatically every time you open a shall
echo "source /opt/ros/[ROS_version]/setup.bash" >> ~/.bashrc
# For example, if you are using kinetic version of ROS
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Source setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Package Modules
---
### 3.1 Nodes
- `uarm_core.py` is the main node. Run this node before anything else. This node has two main modes: **Control-Mode** and **Monitor-Mode**. **Control-Mode** is used to control uarm directly in this node. **Monitor-mode** is to subscrib/listen to all topics which can be used to control uarm through these nodes. This node will automatically load **Control-Mode** first.

    **Step 1**: Connect uArm

    Set up ROS enviroment at first
    ```bash
    roscore
    ```
    Open another shall to connect uArm before use.
    ```bash
    rosrun uarm uarm_core.py connect  // this will find uarm automatically
    ```

    **Step 2**: Control-Mode

    Once connect uArm, you can use commands to control. Input `h` to see all the commands
    ```bash
    # For example: attach uarm (use at in short for attach)
    Input Commands (Input h to see all commands):  attach # or at
    # For example: read current x,y,z (use cc in short for currentCoords)
    Input Commands (Input h to see all commands):  currentCoords # or cc
    # For example: move uarm x,y,z (use mt in short for moveTo)
    Input Commands (Input h to see all commands):  moveTo 12 -12 12
    ```
    Input `l` to exit control-mode and get into Monitor mode

    **Step 3**: Monitor-Mode

    If you get the information as below, you can use both Topics-Pub and other ROS Nodes to control uarm through ROS.
    ```bash
    Begin monitor mode - listening to all fucntional topics
    =======================================================
             Use rqt_graph to check the connection         
    =======================================================
    ```

- `uarm_status_node.py` is the node which can control the attach-status or detach-status of uArm.

    Open another shall, and use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # attach uarm
    rosrun uarm uarm_status_node.py attach
    # detach uarm
    rosrun uarm uarm_status_node.py detach
    ```

- `pump_node.py` is the node which can control the pump on or off.

    Use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # pump on
    rosrun uarm pump_node.py on
    # pump off
    rosrun uarm pump_node.py off
    ```

- `report_angles_node.py` is the node which will report current angles.

    Use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # report once
    rosrun uarm report_angles_node.py
    # report 10 times
    rosrun uarm report_angles_node.py 10
    # report 10 times per 2 time_sec
    rosrun uarm report_angles_node.py 10 2
    ```

- `report_coords_node.py` is the node which will report current coords.

    Use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # report once
    rosrun uarm report_coords_node.py
    # report 10 times
    rosrun uarm report_coords_node.py 10
    # report 10 times per 2 time_sec
    rosrun uarm report_coords_node.py 10 2
    ```

- `report_stopper_node.py` is the node which will report stoppper status.

    Use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # report once
    rosrun uarm report_stopper_node.py
    # report 10 times
    rosrun uarm report_stopper_node.py 10
    # report 10 times per 2 time_sec
    rosrun uarm report_stopper_node.py 10 2
    ```

- `write_angles_node.py` is the node which will control 4 servo angles.

    Use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # write 4 servo angles
    rosrun uarm write_angles_node.py 90 50 50 10
    ```

- `move_to_node.py` is the node which will move to x,y,z position.

    Use this node in the **monitor-mode** of `uarm_core.py` node
    ```bash
    # move to x,y,z
    rosrun uarm move_to_node.py 12 -12 12
    # move to x,y,z,time (point 12 -12 12 in 2 seconds)
    rosrun uarm move_to_node.py 12 -12 12 2
    # move to x,y,z,time,servo_4 angle (servo_4 angle is 54)
    rosrun uarm move_to_node.py 12 -12 12 2 54
    ```
### 3.2 Topics

- `uarm_status` - control uarm status.
    ```
    Message_type: `std_msgs/String`.
    Data: attach / detach
    ```
- `pump_control` - control pump on or off.
    ```
    Message_type: `std_msgs/UInt8`.
    Data: 1 / 0
    ```
- `pump_str_control` - control pump on or off.
    ```
    Message_type: `std_msgs/String`.
    Data: high / low
    ```
- `read_coords` - report coords. Coords will also be written in parameters
    ```
    Message_type: `std_msgs/Int32`.
    Data: int (read times).
    Set-param: 'current_x' 'current_y' 'current_z' ('write in ros-parameter and can be invoked')
    ```
- `read_angles` - report angles. Angles will also be written in parameters
    ```
    Message_type: `std_msgs/Int32`.
    Data: int (read times).
    Set-param: 's1' 's2' 's3' 's4' ('will be written in ros_parameter for 4 `servo_angles')
    ```
- `stopper_status` - report stopper status. Status will also be written in parameters
    ```
    Message_type: `std_msgs/Int32`.
    Data: int (read times).
    Set-param: 'stopper_status'
    ```
- `write_angles` - write 4 servo angles.
    ```
    Message_type: uarm/Angles
    Data: int int int int
    ```
- `move_to` - move to x,y,z position.
    ```
    Message_type: uarm/Coords
    Data: float float float
    ```
- `move_to_time` - move to with time.
    ```
    Message_type: uarm/CoordsWithTime
    Data: float float float int
    ```
- `move_to_time_s4` - move to with time and servo_4 angle.
    ```
    Message_type: uarm/CoordsWithTS4
    Data: float float float int int
    ```


### 3.3 Messages
- `uarm/Angles`
    ```
    uint8: servo_1
    uint8: servo_2
    uint8: servo_3
    uint8: servo_4
    ```
- `uarm/Angles`
    ```
    float32: x
    float32: y
    float32: z
    ```
- `uarm/Angles`
    ```
    float32: x
    float32: y
    float32: z
    uint8: time
    ```
- `uarm/Angles`
    ```
    float32: x
    float32: y
    float32: z
    uint8: time
    uint8: servo_4
    ```


## 4. Visualization in RViz
---
### 4.1 Functions

- Display -- display.launch: This function will display robot movement in realtime when you manually move uArm
- Control -- control.launch: This function will allow you control the end-effector movement in 3 DOF along x,y,z axis.

### 4.2 Launch and Run
-**Step 1**: Set up ROS enviroment in **one** shall
```
roscore
```
In the **second** shall, connect uArm and set the listen mode as shown above
```
rosrun uarm uarm_core.py connect  // connect uArm
l                                 // transfer to monitor mode
```
-**Step 2**: Launch

a) For visualization function, in the **third** shall, run

    roslaunch uarm display.launch

b) Or for control function, in the **third** shall, run

    roslaunch uarm control.launch

-**Step 3**: Display and control:
Open rviz to view robot in the **fourth** shall
```
rosrun rviz rviz
```
For both functions, import robot model in "Displays" panel on the left:

```
Add -> RobotModel           // click "add" and choose "RobotModel"
set Cell Size -> 0.1        // change "Cell Size" to 0.1 in "Grid"
set Fixed Frame -> base     // change "Fixed Frame" to base in "Global Options"
```
a) For Display function, right now a robot will display in the main window

b) For Control function, stop Display function and set

```
Add -> InteractiveMarker                  // click "add" and choose "InteractiveMarkers"
Update Topic -> /uarm_controller/update   // change "Update topic" in "InteractiveMarkers"
```
Drag 3 pairs of arrows to control uArm along x, y, z axis.
