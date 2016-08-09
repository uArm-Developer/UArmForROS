# UArmForROS
This is the uarm ROS package designed by Joey Song ( joey@ufactory.cc / astainsong@gmail.com)

## 0. Quickstart and Execution
---
<img src="http://developer.ufactory.cc/images/visualization-visual.png" alt="unpack" style="width: 640px;”/>


## 1. Installation
---
### 1.1 Pre-Requirements
Connect uArm to computer and upgrade your uArmProtocol Firmware
```bash
$ python -m pyuarm.tools.firmware_helper
```
For using this package, the [pyUarm](https://github.com/uArm-Developer/pyuarm) library **SHOULD** be installed first.
```bash
$ pip install pyuarm
```
Make sure your uarm has been calibrated in [THIS WAY](http://developer.ufactory.cc/quickstart/). Otherwise servos may be **BURNED** !!!

### 1.2 Package Download and Install
Install ros package in your src folder of your Catkin workspace.
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/uArm-Developer/UArmForROS.git
$ cd ~/catkin_ws
$ catkin_make
```
## 2. Source FIles
---
Before you use any packages in uarmForROS, source all setup.bash file which allow you access to uarm package
 ```bash
# Using corresponding version of ROS
source /opt/ros/[Ros_version]/setup.bash
# for example, if you are using kinetic version fo Ros
source /opt/ros/kinetic/setup.bash

# source setup.bash when you open new terminal
source ~/catkin_ws/devel/setup.bash
```

## 3. Package Modules
---
### 3.1 Nodes
- `uarm_core.py` is the main node. Run this node before anything else. This node has two main modes: **Control-Mode** and **Monitor-Mode**. **Control-Mode** is used to control uarm directly in this node. **Monitor-mode** is to subscrib/listen to all topics which can be used to control uarm through these nodes. This node will automatically load **Control-Mode** first. 

    **Step 1**: Connect Uarm
    
    Open ROS firstly
    ```bash 
    roscore  // set up ROS enviroment
    ```
    Open another terminal and connect Uarm before use
    ```bash 
    rosrun uarm uarm_core.py connect  // this will find uarm automatically
    ```
    
    **Step 2**: Control-Mode
    
    Once connect uarm, you can use commands to control. Input `h` to see all the commands
    ```bash 
    # For example: attach uarm (use at in short for attach)
    Input Commands (Input h to see all commands):  attach # or at
    # For example: read current x,y,z (use cc in short for currentCoords)
    Input Commands (Input h to see all commands):  currentCoords # or cc
    # For example: move uarm x,y,z (use mt in short for moveTo)
    Input Commands (Input h to see all commands):  mt 12 -12 12
    ```
    Input `e` to exit control-mode and get into Monitor mode
    
    **Step 3**: Monitor-Mode
    
    If you get the information as below, you can use both Topics-Pub and other ROS Nodes to control uarm through ROS.
    ```bash
    Begin monitor mode - listening to all fucntional topics
    =======================================================
             Use rqt_graph to check the connection         
    =======================================================
    ```
    
- `uarm_status_node.py` is the node which can control the attach-status or detach-status of uarm. 
    
    Open another terminal, and use this node in the **monitor-mode** of `uarm_core.py` node
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

- Visualization -- display.launch: This function will display robot movement in realtime when you manually move uArm
- Control -- control.launch: This function will allow you control the end-effector movement in 3 DOF along x,y,z axis.

### 4.2 Launch and Run
-**Step 1**: Connect uarm in ROS enviroment
```
roscore    // set up ROS enviroment in one terminal
```
In the second terminal, connect uarm and set the listen mode as shown above
```
rosrun uarm uarm_core.py connect  // connect uarm 
e                                 // transfer to listen mode
```
-**Step 2**: Luanch
 
a) For visualization function, in the third termianl, run

    roslaunch uarm display.launch

b) Or for control function, in the third termianl, run

    roslaunch uarm control.launch

-**Step 3**: Display and control:
Open rviz to view robot in the fourth terminal
```
rosrun rviz rviz
```
For both functions, import robot model in "Displays" panel: 

```
Add  -> RobotModel          // click "add" and choose "RobotModel"
set Cell Size -> 0.1        // change "Cell Size" to 0.1 in "Grid"
set Fixed Frame -> base     // change "Fixed Frame" to base in "Global Options"
```
a) For visualization function, right now a robot will display in the main window

b) For control function, 

```
Add -> InteractiveMarker                  // click "add" and choose "InteractiveMarkers"
Update Topic -> /uarm_controller/update   // change "Update topic" in "InteractiveMarkers"
```
Drag 3 pairs of arrows to control uarm along x, y, z axis.





