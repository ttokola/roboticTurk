# ROS Catkin workspace for roboticTurk

This folder contains catkin workspace, which is used to develop, manage and build ROS packages.

Some tutorials for actually using Catkin can be found from [here.](http://wiki.ros.org/catkin/Tutorials)

## Getting started

This workspace contains curretly modified version of the [Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/).

This package is mainly required for controlling the actuators in robotic arm. This package has another Dynamixel package [Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) as mandatory dependecy as well, and it might be needed to be edited as well.

To get the robotic arm to work, there are other dependencies and packages that need to be installed first.

**Ubuntu 16.04** is required as operating system, if not ARM based computer is used.
If ARM based computer is used, [here are some images for it.](https://downloads.ubiquityrobotics.com/pi.html)

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
This should move the first actuator to position 2048. What that position is, depends on the value range of actuator.

### Problems and modifications in Dynamixel Workbench: future ideas

As we are using two different types of actuators in the robotic arm, the default Dynamixel controller will not work as excepted.

Controller file can be found from [here.](src/dynamixel-workbench/dynamixel_workbench_controllers/src/dynamixel_workbench_controllers.cpp)

Default controller is using [SyncRead](http://emanual.robotis.com/docs/en/dxl/protocol2/#sync-read) and [SyncWrite](http://emanual.robotis.com/docs/en/dxl/protocol2/#sync-write) for reading/writing data for actuators. 

This means, that it is *reading and writing* data from/into **same** memory address always. As we have two different kind of actuators, these memory addresses are varying. XL320 is not supporting Inderct Addresses so we can't go around this.

To overcome this problem, SyncRead has been replaced with BulkRead, where every address can be speficied. **Downside is**, that packet length will increase, and it takes more bandwith (baudrate).

We tried to replace SyncWrite with BulkWrite as well, but if we are connecting more than 4 actuators, it will start to 'lag', for some reason there is bottleneck somewhere and all data is not processed or moving. There might have been bug in code as well, try to fix it! This enables usage of 'operator' for all actuators out of the box.

#### Ideas for actually using robotic had to receive coordinate and move piece from place to another?

Contoller has currently subscribed at least for two operations: to reveive joint operations and for arbitrary commands. 

New subsrciber could be added into controller, which is excepting coordinates. Based on these coordinates, new joint operations could be created, and in the end of joint movement, gripper could be controlled separately.

Alternatively the arm could be controlled by using an existing operator and command interface. The actual 'operator' package should be modified to act as subscriber for coordinates. It should create joint operations based on coordinates. In specific positions or time tables, it could control gripper separately.

These are some ideas, there are probably many other ways to implement it!

