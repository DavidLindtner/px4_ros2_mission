# NOT YET FULLY DEVELOPED
# Simulation of Unmanned Aircrafts in a Virtual Environment

For downloading all ROS2 packages run command

```bash
vcs import < vcs.repos
```

In order to install all dependencies for simulation of drone missions with PX4 & ROS2 you need to follow steps in `install.md`

## Run simulation with single wehicle

In order to simulate ROS 2 mission with PX4 firmware you need to run

  * MicroRTPS agent 
  * PX4 simulation firmware with Gazebo simulator
  * ROS 2 node

To run Gazebo simulator with PX4 SITL firmware you need to run this command in PX4 firmware directory downloaded previously.
```bash
make px4_sitl_rtps gazebo
```

To start communication bridge between ROS 2 (DDS) and PX4 (uORB) you need to source ROS 2 workspace (installed according to `install.md`) and then run microRTPS agent.
```bash
source ~/px4_ros2_missions/install/setup.bash
micrortps_agent -t UDP
```

To run ROS2 mission you need to source ROS 2 workspace and than run ROS 2 node.
```bash
source ~/px4_ros2_missions/install/setup.bash
ros2 run px4_missions simpleMission
```
## Run supervised simulation

```bash
ros2 launch px4_missions supervised_launch.py
```
## Run simulation with multiple wehicles with MAVLink

To start multiple wehicle simulation (multiple instances of PX4 firmware) with gazebo simulator run command in PX4-Autopilot directory:
```bash
./Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_default -m iris -n 3
```

Then run ROS 2 launch file which launchs:
* 3x mavros
* 3x simpleMission ROS 2 node

```bash
ros2 launch px4_missions multipleDrone_launch.py 
```

## Run simulation with multiple wehicles with RTPS - NOT YET NOT WORKING

> **ISSUES:**
>
> It is not possible to send commands to second instance of vehicle through RTPS
>
>https://github.com/PX4/PX4-Autopilot/issues/18143
>
>https://discuss.px4.io/t/multi-vehicle-simulation-rtps-gazebo-ros2/26578
>


To start multiple wehicle simulation (multiple instances of PX4 firmware) with gazebo simulator run command in PX4-Autopilot directory:
```bash
./Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_rtps -m iris -n 4
```

To start communication bridge between ROS 2 (DDS) and PX4 (uORB) you need to run microRTPS agent for all 4 wehicles with defined MAVlink ports. Each microRTPS aget run in separed terminal.
```bash
source ~/px4_ros2_missions/install/setup.bash
micrortps_agent -t UDP -r 2020 -s 2019 -n vhcl0
micrortps_agent -t UDP -r 2022 -s 2021 -n vhcl1
micrortps_agent -t UDP -r 2024 -s 2023 -n vhcl2
micrortps_agent -t UDP -r 2026 -s 2025 -n vhcl3
```

To run ROS2 mission you need to source ROS 2 workspace and than run ROS 2 node.
```bash
source ~/px4_ros2_missions/install/setup.bash
ros2 run px4_missions multipleMission
```

> **NOTE:**
>
> This tutorial starts simulation with 4 drones in Gazebo simulator
> 