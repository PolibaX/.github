# PoliBaX Drone Contest

Public README for Leonardo Drone Contest Project.

## Simulation
This section contains progress about simulation.

### Isaac Sim
- *Progress*: 20%
- *Status*: Needs assets ⚙️
- *Working members*: Antonio Pio Maggio, Gabriele Santangelo
- *Repo*: [isaac-sim](https://github.com/PolibaX/isaac-sim)

This repository contains links to tutorials to get Isaac up and running on docker and will contain digital twins (in .usd or .urdf format) for the project.

## Software
This section contains progress about hardware.

### PX4 mavros Relay (Drone)
- *Progress*: 80%
- *Status*: Needs review 📄
- *Working members*: Walter Brescia, Nunzio Barone, Antonio Pio Maggio, Gabriele Santangelo
- *Repo*: NONE

Software needed to transmit computed pose and setpoint pose to PX4.

> No specific repository yet.
> (Should this also contain mavlink and mavros itself or should those be separated?)

### ODrive Control (Rover)
- *Progress*: 100%
- *Status*: Done ✅
- *Working members*: Walter Brescia
- *Repo*: [*odrive_ctrl*](https://github.com/PolibaX/odrive_ctrl)

Low-level controller for an odrive-based diff drive, in the form of a ROS2 package.

### Maxon Control (Rover)
- *Progress*: 0%
- *Status*: Waiting for assignment 🔎
- *Working members*: Nobody
- *Repo*: NONE

Low-level controller for maxon motors in diff drive configuration, in the form of a ROS2 package.

### Path Planner
- *Progress*: 20%
- *Status*: First Implementations ⚙️
- *Working members*: Ivan Cisternino, Francesco Piscopo
- *Repo*: NONE

Local and Global path planners for both rover and drone, in the form of a ROS2 package containing multiple nodes.

### Mission Planner
- *Progress*: 5%
- *Status*: Research phase 📖
- *Working members*: Antonio Pio Maggio, Gabriele Santangelo, Ivan Cisternino, Francesco Piscopo
- *Repo*: NONE

High level mission planner for both rover and drone, in the form of a ROS2 package containing one or multiple nodes.

### Map Aligner
- *Progress*: 5%
- *Status*: Implementation phase ⚙️
- *Working members*: Antonio Pio Maggio
- *Repo*: NONE

Global map aligner on aruco anchors and fine adjustments. This node will provide a shared reference frame for all the agents.

### Map Merger
- *Progress*: 5%
- *Status*: Research phase 📖
- *Working members*: Antonio Pio Maggio
- *Repo*: NONE

Global map merger. This node will provide a global octomap for navigation purposes
and a global point cloud for visialization purposes, in the form of a ROS2 package
containing one or multiple nodes.

### ROS2 Aruco
- *Progress*: 90%
- *Status*: Fully working, needs refinements 👍
- *Working members*: Antonio Pio Maggio
- *Repo*: [ros2-aruco](https://github.com/PolibaX/ros2-aruco)

ROS2 Wrapper for OpenCV Aruco Marker Tracking. This also does Pose Filtering and Accumulation.

### Ground Station UI
- *Progress*: 70%
- *Status*: Widgets in developement 🚧
- *Working members*: Antonio Pio Maggio, Gabriele Santangelo
- *Repo*: [ground-station-ui](https://github.com/PolibaX/ground-station-ui)

Ground Station UI for displaying telemetry data from the system, camera feed, ambient mapping, and node status.
This also allows mission control and status display.

### ROS2-GCS Bridge
- *Progress*: 80%
- *Status*: New protocols in developement 🚧
- *Working members*: Gabriele Santangelo, Antonio Pio Maggio
- *Repo*: [ros2-gcs-bridge](https://github.com/PolibaX/ros2-gcs-bridge)

Bridge from ROS2 to Ground Control Station and back via WebSocket protocol.

### ZED WebRTC
- *Progress*: 80%
- *Status*: Working not complete 👍
- *Working members*: Antonio Pio Maggio
- *Repo*: [ZED-WebRTC](https://github.com/PolibaX/ZED-WebRTC)

Tool to stream ZED's video feed (or any other video feed) over the network using the WebRTC protocol.

## Hardware
This section contains progress about hardware.

### Drone

#### V1.1 - Chotto
- *Progress*: 50%
- *Status*: Waiting for parts 📦
- *Description*: Power Balanced version of Chotto. No changes to configuration.
- *Additions*: 2812 900kv Motors, 6s 35A ESCs
- *Drawbacks*: - Needs testing -

#### V1.0 - Chotto
- *Progress*: 100%
- *Status*: Done ✅
- *Description*: First version fully working hardware. Holybro X500 v2 Quadrotor 4S.
- *Additions*: ZED2i, Jetson Xavier NX
- *Drawbacks*: Underpowered motors, this version hovers around 70% throttle.

### Rover

#### V2.0 - Pluto (Temporary name)
- *Progress*: 50%
- *Status*: Waiting for parts 📦
- *Description*: Whole new rover made for modularity and ease of customization.
- *Additions*: ZED2i, Jetson Orin AGX, Maxon Motors, Lidar, RGB Cameras.
- *Drawbacks*: - Needs testing -

#### V1.1 - Uranus
- *Progress*: 25%
- *Status*: Part design ⚙️
- *Description*: Transmission fixed version of Uranus.
- *Additions*: New transmission gear parts with wheel locks.
- *Drawbacks*: Overpowered motors. Difficult to customize.

#### V1.0 - Uranus
- *Progress*: 100%
- *Status*: Done ✅
- *Description*: First version partially working hardwaee. Self built frame made of soft wood. Differential drive 3x2 with transmission.
- *Additions*: ZED2i, Jetson Orin AGX, Odrive Motors, Lidar.
- *Drawbacks*: Overpowered motors. Slippery unreliable transmission. Difficult to customize.
