# Behavior Tree Compatible Universal Robot RTDE Nodes (C++)

This repository provides **behavior tree-compatible RTDE (Real-Time Data Exchange) nodes** for **Universal Robots** in **C++**. These nodes enable seamless integration of Universal Robots into robotic applications that utilize behavior trees, allowing developers to create complex, flexible, and modular control strategies in a C++ environment.

For users interested in a **ROS2-compatible behavior tree workspace**, please visit our [btcpp_ros2_ws repository](https://github.com/mdirzpr-sqrpnpr/btcpp_ros2_ws) which provides a compatible environment for working with behavior trees alongside ROS2.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Setting Up the Behavior Tree](#setting-up-the-behavior-tree)
  - [Using RTDE Nodes](#using-rtde-nodes)
- [Developers](#developers)

## Introduction

This project is aimed at developers looking to control Universal Robots using **behavior trees** in **C++**. By utilizing RTDE (Real-Time Data Exchange), these nodes enable real-time communication and control, making them suitable for a wide range of industrial and research applications where robot autonomy and decision-making are required.

## Features

- **Behavior Tree Compatibility**: Easily integrate with C++ behavior tree frameworks to manage complex robot behaviors.
- **Real-Time Data Exchange (RTDE)**: Provides high-frequency data streaming and control with Universal Robots.
- **Modular Nodes**: Use specific RTDE nodes for various robot actions, such as motion control, feedback monitoring, and event handling.
- **Flexible Configuration**: Supports customizable parameters to adapt to various Universal Robots models and behavior tree structures.

## Prerequisites

To use this repository, you will need the following:

1. **Universal Robots** with RTDE support (e.g., UR3, UR5, UR10 models).
2. **Behavior Tree Library in C++** (e.g., [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)).
3. **RTDE Interface Library for C++**: Install the C++ RTDE client library from Universal Robots, such as `ur_rtde` (C++ version).

### Example of Installing RTDE Library

You may install the `ur_rtde` C++ library according to the instructions provided by Universal Robots (You can visit this [Link](https://sdurobotics.gitlab.io/ur_rtde/) also). Typically, you will need to clone and build it as follows:

```bash
git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
cd Universal_Robots_Client_Library
mkdir build && cd build
cmake ..
make
sudo make install
```

## Installation

1. Clone this repository:

```bash
git clone https://github.com/mdirzpr-sqrpnpr/behaviortree_ur_rtde
cd behaviortree_ur_rtde
```
2. Build the project:

```bash
mkdir build && cd build
cmake ..
make
```
3. Make sure that `ur_rtde` and the behavior tree libraries are linked correctly. Update 'CMakeLists.txt' if necessary to include the correct library paths.

## Usage
### Setting Up the Behavior Tree

1. Define Behavior Tree: Write your custom behavior tree using the C++ behavior tree framework.
2. Add RTDE Nodes: Integrate the provided RTDE nodes into the tree, each representing a specific robot action or sensor reading.

### Using RTDE Nodes

These nodes provide predefined actions and conditions compatible with RTDE, such as:

* Move Node: Controls basic motion commands for the robot (e.g., move to position).
* Gripper Control Node: If a gripper is attached, this node can manage its state (open/close).
* Data Monitoring Node: Monitors sensor values or specific conditions.
* Event Node: Triggers actions based on real-time conditions (e.g., if temperature exceeds a threshold).


## Developers

- mdirzpr
- sqrpnpr
