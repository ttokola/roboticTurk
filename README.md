# roboticTurk

This repository is for Embedded Software Project 521275A 2019 (Sulautettujen ohjelmistojen projekti, 2019) in the University of Oulu.

## Introduction

The course aims to provide famous (or infamous) *Automaton Chess Player* as known as the [The Turk](https://en.wikipedia.org/wiki/The_Turk) in a bit more modern and sophisticated ways.

Groups of students will work on subprojects to produce different kind of functionalities for overall project.

Later, these projects are merged and hopefully we will see something that is able to play chess!

## Subprojects

Robotic Turk is produced by combination different kind of components, which are providing atleast:


  * [K.O.U.R.A. (the robotic arm)](docs/KOURA.md)
    * Aims to be the component capable of moving chesspieces from place A to place B.     The first version of robotic arm is provided by student-workers for the course.

  * The Head 
    * Containing camera and screen. Capable of moving itself and providing/showing different kind of information
  * Machine Vision
    * Based on information provided by camera; to produce information about the chesstable, chesspieces and the position of robotic arm
  * Magnetic Recognizion
    * Different kind of approach for detecting the positions chesspieces, based on magnetic fields.
  * Movement Controller
    * Aims to provide movement for robotic arm, based on different kind of information.
  * Overall A.I.
    * Aims to provide artificial intelligence for moving the chess pieces **and** for overall system make system act as whole. 

*TBA later: More projects and components. More information about subprojects. Current details might change. Something might be missing already...*

## ROS

[The Robotic Operating System](https://www.ros.org) will be/has been used to implement the functionalities and combine them working as whole.

Service oriented framework allows development of individual components and controlled communicating between them. 

More precisely,  [Kinetic version](http://wiki.ros.org/kinetic) is recommended version to use in this project, as there are working support for Raspberry Pi devices.

[Ubiquity Robotics is maintaining](https://downloads.ubiquityrobotics.com/pi.html) Raspberry Pi images containing required packages in easy way.




