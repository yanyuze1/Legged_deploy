<h1 align="center">legged_deploy</h1>

<p align="center">
  <a href="readmd_CN.md">中文</a> | <a href="readmd.md">English</a>
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

# Validation

## Unitree Go2

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

# BPX

- MuJoCo Sim-to-Sim

```bash
./cmake_build/bin/rl_sim_mujoco bpx scene
```

![BPX MuJoCo simulation](<images/2026-10-10 21-26-29.gif>)

- Real BPX

```bash
./cmake_build/bin/rl_real_bpx --control 10.21.20.1
```



# Acknowledgments

- [mirrorme_rl_train](https://github.com/mirrormerobotics/mirrorme_rl_train)
- [rl_sar](https://github.com/fan-ziqi/rl_sar/tree/main)
- [robot_lab](https://github.com/fan-ziqi/robot_lab)
