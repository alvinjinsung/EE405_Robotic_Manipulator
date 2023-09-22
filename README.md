# EE405_Robotic_Manipulator

## Lab1: Embedded Board and Development Environment Setup
Established and tested a cross-compiling system for developing embedded systems on a PC-based environment

Various aspects such as Ubuntu Linux for PC, Debian Linux for Beaglebone, CMake, and cross-compile, NFS, and GUI tool(VSCode) tested

## Lab2: Switch Circuit Control
Control of solenoid magnet using GPIO hardware in various ways(command lines using sysfs, shell script and C++ program) tested

## Lab3: Motor Control
Utilized ROBOTIS' Dynamixel to receive encoder data nd implemented the feedback position control

## Lab4: Communication and Camera
Using socket programming, implemented a remote video streaming system using webcam on the Bone, and designed keyboard commander for Beaglebone on PC

## Lab5: Robot Manipulatot
Implemented a system to control 4 DOF robot manipulator in joint space and task space

1) Direct teaching: allowed robot to follow the user-defined waypoints
2) teleoperation control: control of the end-effector position based on the keyboard input from the user

## Lab6: System Integration
Integrating all the previous labs, built a robot manipulation system + teleoperation in a FSM style

1) Composed of two computers: PC(user interface), BBB(interacting with the environment)
2) Vision: During execution, task scene is obtained by BBB(cam, communication)
3) UI, Teleoperation: The user commands where to pick and place (UDP, CLI interface)
4) Teaching, Trajectory generation, Control: BBB can automatically execute predefined motions (interpolation, UDP, state machine, control)


