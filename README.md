# ROS2 Drone Simulation and Control

This ROS2 project includes two nodes: `BaseStationNode` and `DroneNode`, which together simulate the control and movement of a drone. The project utilizes ROS2 for messaging and RViz for visualization.

## Overview

- **BaseStationNode:** This node publishes velocity commands to control the drone's linear and angular movements.
- **DroneNode:** This node simulates the drone's position and orientation, broadcasting its transformation relative to a world frame.

## Prerequisites

- ROS2 (Foxy or later)
- RViz (for visualization)

## Installation

1. Clone the repository into your ROS2 workspace's `src` directory.
2. Navigate back to your ROS2 workspace root and build the project with `colcon build`.
3. Source the workspace with `source install/setup.bash`.

## Running the Simulation

1. Start the BaseStationNode:
2. In a separate terminal, start the DroneNode:


## Visualizing with RViz

1. Launch RViz:
2. In RViz, add a `TF` display to visualize the drone's transformation.
3. Optionally, add other displays as needed to visualize different aspects of the drone's movement and environment.

## Customization

- You can modify the `BaseStationNode` to change the drone's command velocities.
- Adjust the `DroneNode` to simulate different movement patterns.

## Troubleshooting

- Ensure all ROS2 dependencies are installed.
- Verify that your ROS2 workspace is correctly sourced.
- Check RViz configurations and ensure the correct frame is selected.

## Contributing

Contributions to enhance or extend the functionality of this project are welcome. Please follow the standard ROS2 development practices for contributions.

