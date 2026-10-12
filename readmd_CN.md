<h1 align="center">legged_deploy</h1>

<p align="center">
  <a href="readme_CN.md">中文</a> | <a href="README.md">English</a>
</p>

这是一份四足机器狗强化学习mujoco仿真验证和真机部署项目，[legged_rl](https://github.com/yanyuze1/legged_rl)和[robot_lab](https://github.com/fan-ziqi/robot_lab)项目进行的强化学习训练可在本项目中进行unitree go2d的mujoco仿真测试和真机部署验证。[mirrorme_rl_train](https://github.com/mirrormerobotics/mirrorme_rl_train)项目可进行BPX的强化学习训练。

# 项目机制图

# 快速开始
- 获取依赖
在项目开始前确保相关依赖被正确获取。
```bash
cd legged_deploy
bash library/download_inference_runtime.sh all
bash library/download_mujoco.sh
```
- 编译
```bash
bash build.sh real     # 真机部署，包含键盘速度控制
bash build.sh mujoco   # MuJoCo 仿真
bash build.sh all      # 同时编译；无参数也使用此模式
```
## 使用
### Unitree go2
- Mujoco 2sim

    在进行真机部署前推荐使用Mujoco进行2sim仿真验证确保训练策略在非训练环境下演示效果。
```bash
./cmake_build/bin/rl_sim_mujoco go2 scene
```
![alt text](<images/2026-10-10 10-24-52.gif>)

- Real Unitree go2

    有关Unitree部署需要对网口进行查询，完成网口查询后进行相关控制连接。进行真机部署控制是注意确保机器安全且确保已经使用手机app关闭go2的运动控制模式以免出现控制冲突。
```bash
ifconfig        # 查询unitree相关连接网口信息
./cmake_build/bin/rl_real_go2 enp109s0
```
![alt text](images/0781e5afe740479a3f4b394aced23544.gif)

### BPX
- Mujoco 2sim
```bash
./cmake_build/bin/rl_sim_mujoco bpx scene
```
![alt text](<images/2026-10-10 21-26-29.gif>)

- Real BPX
```bash
./cmake_build/bin/rl_real_bpx --control 10.21.20.1
```
![alt text](images/4726c3bb5c4539193725a07381c2b332.gif)

## 按键控制
### 键盘按键

| 按键 | 功能 | 生效条件 | MuJoCo | Go2 真机 | BPX 真机 |
|---|---|---|:---:|:---:|:---:|
| `0` | 进入起立状态 `GetUp` | 通常从 Passive 开始；从 RL/GetDown 也可重新起立 | ✓ | ✓ | ✓ |
| `1` | 启用强化学习运动策略 `RL Locomotion` | 必须先完成起立动作 | ✓ | ✓ | ✓ |
| `9` | 进入趴下状态 `GetDown` | 起立完成或正在 RL 控制时 | ✓ | ✓ | ✓ |
| `P` | 立即请求 Passive 被动阻尼模式 | GetUp、RL、GetDown 中均有效 | ✓ | ✓ | ✓ |
| `Ctrl+C` | 退出程序并执行清理 | 任意状态 | ✓ | ✓ | ✓ |

| 按键 | 内部变化 | 机器人运动含义 |
|---|---:|---|
| `W` | `vx += 0.1` | 增大前进速度 |
| `S` | `vx -= 0.1` | 减小前进速度/增加后退速度 |
| `A` | `vy += 0.1` | 增大左向横移速度 |
| `D` | `vy -= 0.1` | 增大右向横移速度 |
| `Q` | `wz += 0.1` | 增大正向偏航角速度，通常为左转 |
| `E` | `wz -= 0.1` | 增大负向偏航角速度，通常为右转 |
| `Space` | `vx=vy=wz=0` | 速度指令清零、停止行走 |

### Unitree Go2手柄
Unitree Go2官方手柄仅在Unitree Go2真机部署上作用，mujoco测试和bpx真机不作用。

| Unitree 手柄按键 | 功能 | 生效条件与说明 |
|---|---|---|
| `A` | 起立，进入 `GetUp` | 从 Passive、GetDown 或 RL 状态进入起立流程 |
| `R1 + 方向键上` | 启用强化学习行走策略 | 需要等待起立动作完成后使用 |
| `B` | 趴下，进入 `GetDown` | 起立完成或正在 RL 行走时有效；完成后自动进入 Passive |
| `L1 + X` | 进入 Passive 被动阻尼 | 优先级最高；清零速度并退出 RL 控制 |
| `X` | 切换导航模式 | 默认非 ROS 构建中只切换标志，不会实际接管速度 |

| 手柄输入 | 控制量 | 作用 |
|---|---|---|
| 左摇杆向前/向后 | `vx` | 控制前进和后退 |
| 左摇杆向左/向右 | `vy` | 控制左右横移 |
| 右摇杆向左/向右 | `wz` | 控制左转和右转 |
| 摇杆全部回中 | 速度清零 | 停止继续行走，但机器人仍保持 RL 站立 |
| `L1 + X` | 全部速度清零 | 同时退出策略并进入被动阻尼 |

### 游戏手柄 

# 致谢
- [mirrorme_rl_train](https://github.com/mirrormerobotics/mirrorme_rl_train)
- [rl_sar](https://github.com/fan-ziqi/rl_sar/tree/main)
- [robot_lab](https://github.com/fan-ziqi/robot_lab)