# Real Time Distributed Simulation of a Mobile Robot

## Introduction

This project implements the distributed simulation of a mobile robot using a real time system.
The processes are separated in 3 parts:
- Plant: Implements the dynamic equations to simulate the robot;
- Control: Implements the control signals and
- Adjust: Implements the threads to adjust the controller's parameters.

The simulation is done using a modified Linux Kernel with [RTAI](https://www.rtai.org/) patches.

This project explores concepts such as:
- Real Time Threads using RTAI LXRT, controlling the threads at user space;
- The use of shared resources and how to protect critial regions using semaphores and monitors;
- Shared memory and message exchange across different processes and real-time threads.


## Quick How To


To use it just type:

make;

and then;

./control/control
and in a second window:
./simulation/simulation

It is also possible to simulate it in two computers. In order to do it, just change the
ip address inside the file rtnetAPI.c, and recompile it;


