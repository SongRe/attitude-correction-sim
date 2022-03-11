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

## Abstracted Controller Implementation

The details and implementation of each controller is abstracted out into the following functions:
* Controller initialization
* Controller update
* Controller reset
* Controller teardown
* Controller output

*The controller output abstraction will likely be removed in future revisions or it will be amended to integrate with additional models for reaction wheels and magnetorquers. The update function combined with additional inclusions to the controller proxy will likely be sufficient to replace this function.*

These functions are provided to the struct `cntrl_inf` and is provided to the controller bridge as defined as the struct `cntrl_bridge`. This object allows a singleton object that can be accessed by the implementation of each controller and is created in the main function of `sim_with_control_viewer`. Note that this bridge is passed to the `ControlPipeline` to be used by the abstracted implementation.

The controller proxy, as defined in `cntrl_proxy`, provides a thread-safe way for the abstracted controller implementation to pull commanded and current attitude data of the simulated rigidbody. It also allows the controller to push the new commanded moment to the physics pipeline to be applied to the rigidbody.

For an example of an implemented controller, refer to the eigenaxis control law implemented in the files `include/algs/control_eigenaxis.h` and `src/algs/control_eigenaxis.c` in the `control/` subfolder.