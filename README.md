# sphero_rvr_ros

[![ROS1 Noetic](https://img.shields.io/badge/ROS1-Noetic-blue)](http://wiki.ros.org/noetic/Installation/Ubuntu)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Open in Visual Studio Code](https://img.shields.io/badge/vscode-dev-blue)](https://open.vscode.dev/AIResearchLab/sphero_rvr_ros)

ROS1 meta package containing an implementation of a Sphero RVR robot using ROS1 and the sphero-rvr-sdk in Python.

## Installation

The RVR python SDK is required to run this package. It can be installed using pip or by cloning the SDK.

## Contents

Some interesting packages in this application are:

- `sphero_rvr_description`: URDF description of the RVR robot.
- `sphero_rvr_driver`: ROS1 node that interfaces with the RVR robot using the sphero-rvr-sdk.
- `sphero_rvr_controllers`: ROS1 node that provides a simple control interface for the RVR robot.
