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
# 验证
## Unitree go2
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

# BPX
- Mujoco 2sim
```bash
./cmake_build/bin/rl_sim_mujoco bpx scene
```
![alt text](<images/2026-10-10 21-26-29.gif>)

- Real BPX
```bash
./cmake_build/bin/rl_real_bpx --control 10.21.20.1
```


# 致谢
- [mirrorme_rl_train](https://github.com/mirrormerobotics/mirrorme_rl_train)
- [rl_sar](https://github.com/fan-ziqi/rl_sar/tree/main)
- [robot_lab](https://github.com/fan-ziqi/robot_lab)