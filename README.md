# Attitude Control

This repository contains the C-code for implementation of the CubeSat attitude controller. Further instructions to this repository will be added as functionality is implemented.

It should be noted that code pertaining to the control algorithms is implemented in pure C-code while other code pertaining to the simulation runner and display is written in C++ for convenience.

## Setup

This guide assumes that you are working on a recent Ubuntu-based Linux distro. If you are attempting to develop and **compile** on Windows you may need to do some additional troubleshooting.

### OpenGL

This library is necessary for visualizing the attitude control simulation. Run the commands listed below to install it onto your system:
```bash
sudo apt-get update
sudo apt-get install libglfw3
sudo apt-get install libglfw3-dev
```

### Catch2 Library

This library is used for unit testing of this project. Follow the git installation directories from this repository [here](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md#installing-catch2-from-vcpkg).

## Building

This project uses CMake and it is recommended to use the Visual Studio Code extension to configure and build the project. VS Code will auto build to the `build/` directory. Install the CMake Tools package provided by Microsoft on the extension marketplace. Use `Cntrl + Shift + P` to access the command menu to view available commands, including CMake commands, present within VS Code.

## Running

Within the `build/` directory, the executables `sim_with_viewer` and `sim_with_control_viewer` can be found. `sim_with_viewer` will show a satellite spinning in place and will not modify speed. `sim_with_control_viewer` has a controller-enabled and the satellite should eventually come to a stop if the controller is configured correctly.