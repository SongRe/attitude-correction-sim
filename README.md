# Attitude Control

This repository contains the C-code for implementation of the CubeSat attitude controller. Further instructions to this repository will be added as functionality is implemented.

It should be noted that code pertaining to the control algorithms is implemented in pure C-code while other code pertaining to the simulation runner and display is written in C++ for convenience.

## Setup

This guide assumes that you are working on a recent Ubuntu-based Linux distro. If you are attempting to develop and **compile** on Windows for some reason, may God help you.

It is necessary to install the Quaternion library used in this repository with the following:
```bash
cd external/
git clone https://github.com/MartinWeigel/Quaternion.git
```

### OpenGL

This library is necessary for visualizing the attitude control simulation. Run the commands listed below to install it onto your system:
```bash
sudo apt-get update
sudo apt-get install libglfw3
sudo apt-get install libglfw3-dev
```