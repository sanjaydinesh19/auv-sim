# Mira Simulator

Please see the [https://github.com/Dreadnought-Robotics/mira_sim/wiki](wiki) for installation & usage instructions.

---

## Overview

The Mira Simulator is a ROS 2–based underwater simulation environment built on top of the Stonefish simulator and integrated with ArduPilot SITL. It enables testing of autonomous underwater vehicle (AUV) behaviors, perception pipelines, and control systems in a simulated environment.

This setup supports:
- Physics-based underwater simulation (Stonefish)
- ArduPilot SITL for vehicle control
- QGroundControl for monitoring and mission control
- Custom ROS 2 perception and control nodes

---

## Running the Simulator

The system requires multiple terminals running simultaneously.

---

### Terminal 1: Launch Simulator (Stonefish + TAC Pipeline)

ros2 launch dnt_simulator tac_pipeline.launch.py

This starts:
- The Stonefish simulation environment
- TAC pipeline configuration
- ROS 2 interfaces for the simulated vehicle

---

### Terminal 2: Start ArduPilot SITL

make sitl

This initializes:
- Software-in-the-loop ArduPilot instance
- Communication with the simulator

---

### Terminal 3: Launch QGroundControl

./QGroundControl.AppImage

This provides:
- Ground control station interface
- Telemetry visualization
- Manual control and mission planning

---

### Terminal 4: Run Perception / Control Node

cd ~/DNT/Sim/mira_sim
source install/setup.bash
ros2 run orca_follower follower_node

This runs:
- Custom ROS 2 node for AUV behavior
- Perception pipeline (e.g., pipeline detection)
- Autonomous control logic

---

## Notes

- Ensure all dependencies are installed and the workspace is built before running.
- Run source install/setup.bash in any terminal where ROS 2 nodes are executed.
- Multiple nodes can be launched depending on the experiment setup.
- Avoid committing large files such as logs, videos, or simulation artifacts to the repository.

---

## Repository Structure (Simplified)

mira_sim/
├── src/
│   ├── pipeline_detector/
│   ├── orca_follower/
│   ├── stonefish_ros2/
├── install/
├── build/
├── log/

---

## Additional Resources

Refer to the wiki for:
- Installation steps
- Dependency setup
- Troubleshooting
- Simulation configurations
