# ROS Catkin workspace for roboticTurk

This folder contains catkin workspace, which is used to develop, manage and build ROS packages.

Some tutorials for actually using Catkin can be found from [here.](http://wiki.ros.org/catkin/Tutorials)

## Getting started

This workspace contains curretly modified version of the [Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

This package is mainly required for controlling the actuators in robotic arm. This package has another Dynamixel package [Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) as mandatory dependecy as well, and it might be needed to edit as well.

To get the robotic arm to work, there are other dependencies and packages to be installed at first.

**Ubuntu 16.04** is required as operating system, if not ARM based computer is used.
If ARM based computer is used, [here are some images for it.]((https://downloads.ubiquityrobotics.com/pi.html))

To install these packages, run:
```shell
sudo sh install_dep.sh
```

This will install ROS and other required packages, and download external packages including that Dynamixel SDK.

After the script has run, we need to 'source' some sources, that shell will indetify some commands.

```shell
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
````
Previously we did not install external packages yet. To do so, we need to simply run command 
```shell
catkin_make
```
This will compile all viable ROS packages from the workspace.

## Movement Controller - Connecting and controlling the arm

Let's except that our circuit and any cables are correctly set.
The arduino board has 'relay' code uploaded and its running.
Power has been connected to five XL430 actuators and for one XL-320 actuator.

Next we need to just connect USB from arduino into the computer, which is actually running the ROS. Once we have done it, we need to set permissions for device by running:

```shell
sudo chmod a+rw /dev/ttyACM0
```

We are going to start the Dynamixel controller: it's responsibility is to control actuators based on 

The idea is identical for using the Dynamixel Controller than it is described in their E-manual [here.](emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#controllers)

Note, that in each terminal, environment must sources as it was previously done.

Note additionally, that namespace is always 'manipulator', not dynamixel_workbench as in examples.

To start controller, run:
```shell
 roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
 ```

 It should be able to find all of the actuators. The configuration file for them can be seen [here.](src/dynamixel-workbench/dynamixel_workbench_controllers/config/koura.yaml)

 Now we should have the controller running. At least in this version, it is not doing anything on its own. It is waiting some instructions to make something for actuators.

 One possibility for moving to use 'operator' as described in e-manual.
 We have configured one example movement for file, which can be found from [here.](src/dynamixel-workbench/dynamixel_workbench_operators/config/demo.yaml)

 To publish operator positions for controller, open new terminal and run:
 ```shell
 roslaunch dynamixel_workbench_operators joint_operator.launch
 ```
 and to actually excute the movement
 ```shell
 rosservice call /manipulator/execution "{}"
 ```

This should start the movement of arm. **Gripper will not move**, as currently it is not possible move with operator.

However, each of the actuators can be controlled via rosservice, to send commands into controlled with following format:
```shell
rosservice call /manipulator/dynamixel_command "command: ''
id: 1
addr_name: 'Goal_Position'
value: 2048"
```
This should move first actuator to position 2048. What that position is, depends on the value range of actuator.

### Problems and modifications in Dynamixel Workbench: future ideas
