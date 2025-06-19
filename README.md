# ROS2 Cartesian to Joint Motion Benchmark (Test Code)

![Status](https://img.shields.io/badge/status-experimental-red)
![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-N/A-lightgrey)

**This project is an internal benchmark and is not yet published or intended for production use.**

## Description

This repository contains test code for benchmarking inverse kinematics (IK) solutions. The primary function of this node is to receive a target Cartesian pose from a ROS2 topic, compute the corresponding joint-space motion, and prepare it for execution on a robot.

It is designed to work in conjunction with a separate ROS2 node that publishes the target poses, such as one using HTC Vive trackers for real-time motion input.

## How It Works

1.  **Subscribes to Target Pose:** The node listens to the `/tarpos_pub` topic for `geometry_msgs/PoseStamped` messages.
2.  **Inverse Kinematics:** Upon receiving a new target pose, it calculates the required joint angles using an inverse kinematics solver.
3.  **Maps Motion:** It maps the continuous Cartesian motion from the input device to discrete or continuous joint motion for a target manipulator.

## Prerequisites

* **ROS2 Humble Hawksbill:** This code has been tested on ROS2 Humble.
* **Target Pose Publisher:** You must have another ROS2 node running that publishes target poses to the `/tarpos_pub` topic. The expected message type is `geometry_msgs/PoseStamped`.
    * For example, you can use a modified `talker.py` from a `vive_ros` package.

## Getting Started

As this project is still in a testing phase, the installation is manual.

### Installation

1.  **Clone the repository** into the `src` folder of your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone [https://github.com/your-username/your-repo-name.git](https://github.com/your-username/your-repo-name.git)
    ```

2.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select your_package_name
    ```

3.  **Source the workspace:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

### Usage

1.  **Launch the target publisher:**
    In a separate terminal, run the node that publishes the target poses.
    ```bash
    # Example using a vive_ros publisher
    ros2 launch vive_ros vive_publisher.launch.py
    ```

2.  **Launch the IK benchmark node:**
    In a new terminal (after sourcing your workspace), launch this node.
    ```bash
    ros2 run your_package_name ik_benchmark_node
    ```

3.  **Observe the output:**
    The node will log the computed joint states or can be configured to publish them to another topic for execution or visualization in tools like RViz2.

## Current Status

* [ ] Core IK solver integration.
* [ ] ROS2 topic subscription and message handling.
* [ ] Basic performance logging.
* [ ] **Not Ready For Production:** This is experimental code. The API and functionality are subject to change without notice.
