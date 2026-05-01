# Mira Simulator

<img width="1228" height="866" alt="image" src="https://github.com/user-attachments/assets/3ae59602-d557-415d-a915-ce183217f785" />

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
```
ros2 launch dnt_simulator tac_pipeline.launch.py
```
This starts:
- The Stonefish simulation environment
- TAC pipeline configuration
- ROS 2 interfaces for the simulated vehicle

---

### Terminal 2: Start ArduPilot SITL
```
make sitl
```
This initializes:
- Software-in-the-loop ArduPilot instance
- Communication with the simulator

---

### Terminal 3: Launch QGroundControl
```
./QGroundControl.AppImage
```
This provides:
- Ground control station interface
- Telemetry visualization
- Manual control and mission planning

---

### Terminal 4: Run Perception / Control Node
```
cd ~/DNT/Sim/mira_sim
source install/setup.bash
ros2 run orca_follower follower_node
```
This runs:
- Custom ROS 2 node for AUV behavior
- Perception pipeline (e.g., pipeline detection)
- Autonomous control logic

---

## Running Pipeline Follower

### Terminal 1 - Launch TAC Pipeline Sim

```
ros2 launch dnt_simulator tac_pipeline.launch.py
```

### Terminal 2 - Run Software In the Loop

```
make sitl
```

### Terminal 3 - Run MIRA Master with venv

```
source .venv/bin/activate
ros2 run mira2_control_master alt_master --ros-args -p pixhawk_address:=tcp:localhost:5760
```

### Terminal 4 - Run Node

```
ros2 run pipeline_follower pipeline_follower_node
```

### Terminal 5 - View Debug Camera

```
ros2 run rqt_image_view rqt_image_view /pipeline_follower/debug_image
```

## Notes

- Ensure all dependencies are installed and the workspace is built before running.
- Run source install/setup.bash in any terminal where ROS 2 nodes are executed.
- Multiple nodes can be launched depending on the experiment setup.
- Avoid committing large files such as logs, videos, or simulation artifacts to the repository.

---

## Repository Structure (Simplified)
```
mira_sim/
├── src/
│   ├── pipeline_detector/
│   ├── orca_follower/
│   ├── stonefish_ros2/
|   ├── pipeline_follower/
├── install/
├── build/
├── log/
```
---

## Additional Resources

Refer to the wiki for:
- Installation steps
- Dependency setup
- Troubleshooting
- Simulation configurations
