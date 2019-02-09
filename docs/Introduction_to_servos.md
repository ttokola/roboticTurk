# The usage of Dynamixel actuators (servos)

*A very short introduction without going into too deep details: how to control actuators.*

## Getting started

There are two types of actuators (servos):

* [Dynamixel XL320](http://emanual.robotis.com/docs/en/dxl/x/xl320/)
    * It has been used for controlling the 'Gripper'
* [Dynamixel XL430-W250-T](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
  * It has been used to move joints

In hardware side, actuators are using TTL Half Duplex Asynchronous Serial Communication
(8bit, 1stop, No Parity)

In data packet side, both of the actuators are using [Dynamixel Protocol 2.0](http://emanual.robotis.com/docs/en/dxl/protocol2/) for communication.

Both of the actuators have multiple memory addresses, and by changing values of these addresses, something intended will happen.

Shortly, by using half-duplex TTL and implementing Dynamixel protocol for data packets to change values in memory addresses, we can succesfully control actuators. 

**Note: Both of the actuators are using 5 voltages** for sending the status packet over TTL!. Pins of Arduino are supporting this, but Raspberry's  pins (excluding 5V VDD) are supporting ***only 3.3 voltages.***

Therefore we must connect actuators to Arduino, unless we are using some kind of voltage regulator.
However, messages can be send into actuator with voltage less than 5.

Overall, **be absolutely certain to not going over the recommend voltage values,** which are used for powering actuators.
  * Max. for XL430-W250: 12V
  * Max. for XL-320: 8.4V

External power supply is required for actuators, as both Raspberry and Arduino are offering 5 voltages at maximum as power source. Ground must be connected together among all power supplies and devices.

## Half-duplex vs full-duplex problem

Current available hardware is based on Arduino Uno and Raspberry Pi 3.

Both of the devices have both RX (receive), and TX (transmission) pins. (Note: Actuators cannot be connected directly into Raspberry 3, since output voltage 5V exceeds the pin voltage 3.3V.)

Both of the different kind actuators have three cables: *ground*, *VDD* and *data*.

Thefore data from the actuators is coming in form of half-duplex (moving into one direction at time).

Some problems will arise, since Arduino and Raspberry Pi are supporting full-duplex directly, unless we somehowe make time-controlled software for single pin.
Otherwise we have to implement half-duplex -> full-duplex converter, in order to send and receive data from actuators. 

*What this means in practise?*

We can't get any responses from actuators without converting the full-duplex into half-duplex and vice versa.

Following circuit must be implemented: [ Connecting the Dynamixel motors to Raspberry pi 3](https://github.com/horverno/deep-pilot/wiki/Electrical-design)

It is based on circuit introduced by Savage Electronics [here.](http://savageelectronics.blogspot.com/2011/01/arduino-y-dynamixel-ax-12.html)

[Here](https://robottini.altervista.org/dynamixel-ax-12a-and-arduino-how-to-use-the-serial-port) is one example implementation on Arduino.

*Circuit has been connected into Arduino on our project as well.* Above solution also uses resistor to lower incoming voltage from actuator, but that is not stable solution.

Circuit is based on usage of so called 'direction pin'. We have to select one pin, and by giving HIGH or LOW power for it, we can actually control the direction of data.

Direction pin has to be connected into pin 19 of 74LS241 component, as shown in above links. When power is HIGH, data output from TX pin (or any pin used as TX pin) will reach data cable of actuator.

When power is set to LOW, data from actuator cable is going to RX pin, and transmission is impossible.

Here is the example image:

TODO added later


## Arduino Relay

Instead of making actuator functionality on Arduino in this project, we made Arduino act as data-relay and controller of the direction of data.

Therefore, we can plug-in USB-cable into the Arduino, and connect the another end of the USB into the Raspberry or another PC to actually control the actuators.

The data what we are sending into Arduino via USB, is behaved same as sending data into physical RX pin.
Data what Arduino is sending, goes for TX pin as well into USB via Serial connection.

Software serial has been used to send data from pin 11 to actuators that Arduino received from RX pin via USB. Before sending data, power for direction pin is set for HIGH, and once the data is sent, power will be set for LOW. This way data is relayed and direction of data is controlled.

Example relay code can be seen [here.](src/arduinorelay.cpp)

**NOTE:** Limits of Software Serial should be noted: it supports only baudrate of 57600 at maximum, and so actuators should be somehow configured to use this baudrate beforehand.

## Actual code for actuators

Both of the following packages are developed and maintained by Robotis, under Apache 2.0 license.

* Dynamixel SDK is used as ROS package in driver level for actuators. There are alot of useful code examples, in case some functionality will be tested or added only for Arduino.
  * GitHub [URL.](https://github.com/ROBOTIS-GIT/DynamixelSDK)
  * Documentation [URL.](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

* Dynamixel Workbench is one level higher environment: it is generating functionality based on driver code.
  * GitHub [URL.](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
  * Documentation [URL.](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)


[Here](https://github.com/hackerspace-adelaide/XL320) is one simpler approach for using XL-320 actuators on Arduino.

By using these ROS packages with combintion of custom made circuit and Arduino data-relay, we are already able to control Dynamixel actuators, with Raspbebery or another device running ROS.



