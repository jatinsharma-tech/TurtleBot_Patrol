# turtlebot_patrol

ROS 2 package that makes a TurtleBot3 (Burger) patrol a square route in Gazebo, avoid obstacles using the LiDAR, stop on an emergency-stop service call, and log telemetry to a CSV file.

Tested on Ubuntu 22.04 + ROS 2 Humble.

## Nodes

| Node | What it does |
|---|---|
| `lidar_obstacle_detector` | Reads `/scan`, splits it into front (-30° to 30°), left (30° to 90°) and right (-90° to -30°), finds the minimum distance in each and publishes `[front, left, right, obstacle_flag]` on `/obstacle_status` |
| `patrol_controller` | State machine with PATROL / AVOID / HALTED. Drives the square using `/odom`, turns toward the more open side when something is closer than 0.5 m in front, and publishes `/cmd_vel` |
| `emergency_stop_server` | `/trigger_estop` service (`std_srvs/SetBool`). `true` = stop, `false` = resume. Publishes the state on `/estop_state` |
| `telemetry_logger` | Total distance, average speed and simulated battery (1% per 10 s of motion). Prints to console, publishes `/robot_telemetry` and writes `telemetry_log.csv` |

![rqt_graph](docs/images/rqt_graph.png)

## Setup

Install the TurtleBot3 simulation packages:

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-msgs
```

Clone into a workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this repo url> turtlebot_patrol
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot_patrol
source install/setup.bash
```

## Launch

```bash
ros2 launch turtlebot_patrol patrol_simulation.launch.py
```

This opens Gazebo with `turtlebot3_world` (the one with the pillars), spawns the robot and starts all 4 nodes with the params from `config/patrol_params.yaml`.

![robot patrolling in gazebo](docs/images/gazebo_patrol.png)

## Usage / Testing

Emergency stop (open a new terminal and source the workspace first):

```bash
# stop
ros2 service call /trigger_estop std_srvs/srv/SetBool "{data: true}"

# resume
ros2 service call /trigger_estop std_srvs/srv/SetBool "{data: false}"
```

![estop service call](docs/images/estop.png)

Other useful commands:

```bash
ros2 topic echo /obstacle_status     # front, left, right, flag
ros2 topic echo /estop_state
ros2 topic echo /robot_telemetry
cat telemetry_log.csv                # created in the folder you launched from
```

## Parameters (`config/patrol_params.yaml`)

- `safe_distance` – obstacle threshold (0.5 m)
- `linear_speed`, `angular_speed` – robot speeds
- `side_length` – size of the square
- `battery_start`, `drain_per_10s`, `log_file`, `log_period` – telemetry settings

## How it works

PATROL: go straight for `side_length` metres (measured from odom), then turn 90°, repeat.
AVOID: if front distance < `safe_distance`, stop, compare left vs right and rotate toward the bigger one until the front is clear, then start a new side of the square from there.
HALTED: when `/estop_state` is true the controller publishes zero velocity every cycle. When it goes false the robot goes back to PATROL.
