<h1 align="center">legged_deploy</h1>

<p align="center">
  <a href="readmd_CN.md">中文</a> | <a href="README.md">English</a>
</p>

This project provides MuJoCo simulation validation and real-robot deployment for quadruped reinforcement learning policies. Policies trained with [legged_rl](https://github.com/yanyuze1/legged_rl) or [robot_lab](https://github.com/fan-ziqi/robot_lab) can be tested on a Unitree Go2 in MuJoCo and then deployed to the real robot. The [mirrorme_rl_train](https://github.com/mirrormerobotics/mirrorme_rl_train) project can be used to train reinforcement learning policies for BPX.

# Project Architecture

# Quick Start

- Install the dependencies

Make sure all required dependencies are installed before building the project.

```bash
cd legged_deploy
bash library/download_inference_runtime.sh all
bash library/download_mujoco.sh
```

- Build

```bash
bash build.sh real     # Real-robot deployment with keyboard velocity control
bash build.sh mujoco   # MuJoCo simulation
bash build.sh all      # Build both targets; this is also the default when no argument is given
```

## Usage

### Unitree Go2

- MuJoCo Sim-to-Sim

  Before deploying to the real robot, it is recommended to validate the trained policy in MuJoCo to ensure that it performs correctly outside the training environment.

```bash
./cmake_build/bin/rl_sim_mujoco go2 scene
```

![Unitree Go2 MuJoCo simulation](<images/2026-10-10 10-24-52.gif>)

- Real Unitree Go2

  To deploy on a Unitree Go2, first identify the network interface connected to the robot, then use it to establish the control connection. Before running the controller, make sure the robot is in a safe environment and use the mobile app to disable the Go2 motion-control mode to prevent control conflicts.

```bash
ifconfig        # Find the network interface connected to the Unitree robot
./cmake_build/bin/rl_real_go2 enp109s0
```

![Real Unitree Go2 deployment](images/0781e5afe740479a3f4b394aced23544.gif)

### BPX

- MuJoCo Sim-to-Sim

```bash
./cmake_build/bin/rl_sim_mujoco bpx scene
```

![BPX MuJoCo simulation](<images/2026-10-10 21-26-29.gif>)

- Real BPX

```bash
./cmake_build/bin/rl_real_bpx --control 10.21.20.1
```

![Real BPX deployment](images/4726c3bb5c4539193725a07381c2b332.gif)

## Controls

### Keyboard Controls

| Key | Function | Activation Condition |
|---|---|---|
| `0` | Enter the `GetUp` state | Normally starts from Passive; can also stand up again from RL or GetDown |
| `1` | Enable the `RL Locomotion` policy | The get-up motion must be completed first |
| `9` | Enter the `GetDown` state | Available after standing up or while under RL control |
| `P` | Immediately request Passive damping mode | Available during GetUp, RL, and GetDown |
| `Ctrl+C` | Exit the program and perform cleanup | Available in any state |
| `W` | `vx += 0.1` | Increase forward velocity |
| `S` | `vx -= 0.1` | Decrease forward velocity or increase backward velocity |
| `A` | `vy += 0.1` | Increase leftward lateral velocity |
| `D` | `vy -= 0.1` | Increase rightward lateral velocity |
| `Q` | `wz += 0.1` | Increase positive yaw rate, normally turning left |
| `E` | `wz -= 0.1` | Increase negative yaw rate, normally turning right |
| `Space` | `vx=vy=wz=0` | Clear all velocity commands and stop walking |

### Unitree Go2 Controller

The official Unitree Go2 controller is supported only when deploying to a real Unitree Go2. It has no effect in MuJoCo simulations or on a real BPX robot.

| Unitree Controller Input | Function | Activation Condition and Description |
|---|---|---|
| `A` | Stand up and enter `GetUp` | Starts the get-up sequence from Passive, GetDown, or RL |
| `R1 + D-pad Up` | Enable the reinforcement learning locomotion policy | Use only after the get-up motion has completed |
| `B` | Lie down and enter `GetDown` | Available after standing up or during RL locomotion; automatically enters Passive when complete |
| `L1 + X` | Enter Passive damping mode | Highest priority; clears all velocity commands and exits RL control |
| `X` | Toggle navigation mode | In the default non-ROS build, this only toggles a flag and does not take control of velocity commands |
| Left stick forward/backward | `vx` | Controls forward and backward motion |
| Left stick left/right | `vy` | Controls lateral motion to the left and right |
| Right stick left/right | `wz` | Controls left and right turns |
| All sticks centered | Zero velocity | Stops walking while the robot remains standing under RL control |
| `L1 + X` | Clear all velocity commands | Exits the policy and enters Passive damping mode |

### Game Controller

| Controller Input | Function | Effect |
|---|---|---|
| `A` | Stand up and enter `GetUp` | |
| `RB + D-pad Up` | Enable the RL locomotion policy | |
| `B` | Lie down and enter `GetDown` | |
| `LB + X` | Enter Passive damping mode | |
| Left stick forward | Positive `vx` | Move forward |
| Left stick backward | Negative `vx` | Move backward |
| Left stick left | Positive `vy` | Move laterally to the left |
| Left stick right | Negative `vy` | Move laterally to the right |
| Right stick left | Positive `wz` | Turn left |
| Right stick right | Negative `wz` | Turn right |
| Sticks centered | Zero velocity | Stop moving while remaining standing |

# Acknowledgments

- [mirrorme_rl_train](https://github.com/mirrormerobotics/mirrorme_rl_train)
- [rl_sar](https://github.com/fan-ziqi/rl_sar/tree/main)
- [robot_lab](https://github.com/fan-ziqi/robot_lab)
