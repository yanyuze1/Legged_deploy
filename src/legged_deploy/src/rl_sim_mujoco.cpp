/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rl_sim_mujoco.cpp
 * @brief MuJoCo 强化学习仿真桥接层的核心实现。
 * 负责统一调度底层的物理引擎渲染、神经网络推理、状态机管理以及硬件 I/O 映射。
 */
#include "rl_sim_mujoco.hpp"

// 用于处理 POSIX 系统信号 (如 SIGINT/Ctrl+C) 的静态实例，确保资源能释放
RL_Sim* RL_Sim::instance = nullptr;
namespace { volatile std::sig_atomic_t simulation_exit_requested = 0; }

/**
 * @brief 构造 RL_Sim 仿真环境，初始化 MuJoCo 上下文，并派生多线程任务。
 * @param argc 命令行参数数量
 * @param argv 期望参数: [1] robot_name (机器人名称), [2] scene_name (场景名称)
 * @throws std::runtime_error 参数错误时抛出异常
 */RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
    }

    if (this->robot_name != "go2" && this->robot_name != "bpx")
    {
        throw std::runtime_error("Supported robot names: go2, bpx");
    }

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
// 拦截 Rosetta 2 运行环境，避免 AVX 指令集转译导致的未定义行为或崩溃
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    // 扫描插件目录中的库以加载更多插件
    scanPluginLibraries();

    // 初始化MuJoCo默认渲染相机与物理微调参数
    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    // 实例化仿真核心，绑定 GLFW 渲染适配器
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../robot_description/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    if (!std::filesystem::is_regular_file(filename))
        throw std::runtime_error("MuJoCo scene not found: " + filename);

    // BPX joins its physics thread before destroying the callback owner.
    // The filename string is copied into the thread, never borrowed from ctor scope.
    if (this->robot_name == "bpx")
    {
        this->bpx_physics_thread_ = std::thread([this, filename]() {
            PhysicsThread(sim.get(), filename.c_str(),
                [this](mjModel* model, mjData* data, bool advance) {
                    return this->BpxPhysicsStep(model, data, advance);
                });
        });
    }
    else
    {
        std::thread([this, filename]() {
            PhysicsThread(sim.get(), filename.c_str(), PhysicsStepHook{});
        }).detach();
    }

    try
    {
    
    // 轮询自旋锁：阻塞主线程，等待物理线程初始化 mj_data 数据结构
    while (1)
    {
        bool data_prepared = false;
        if (this->robot_name == "bpx")
        {
            // PhysicsThread publishes m/d while holding the same sim mutex.
            const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
            if (d)
            {
                this->mj_model = m;
                this->mj_data = d;
                data_prepared = true;
            }
        }
        else data_prepared = (d != nullptr);
        if (data_prepared)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        if (simulation_exit_requested) sim->exitrequest.store(1);
        if (sim->exitrequest.load())
            throw std::runtime_error("MuJoCo load canceled or failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (this->robot_name != "bpx")
    {
        this->mj_model = m;
        this->mj_data = d;
    }

    // 初始化外设操作摇杆，绑定使用js0设备，16位精度，推荐将摇杆设备抽取至 YAML 配置中解耦
    this->SetupSysJoystick("/dev/input/js0", 16); // 16 bits joystick

    // read params from yaml，读取机器人参数
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    // 基于机器人类型，动态应用反射/工厂模式分配有限状态机 (FSM)
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot 初始化机器人关节数量、输出控制维度等
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

    if (this->robot_name == "bpx")
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
        this->ValidateBpxModel();
        this->ResetBpxSimulation(false);
        this->simulation_running = (sim->run != 0);
        this->bpx_ready_ = true;
    }

    // loop 
    /*
    *  控制循环 loop_control：负责读取机器人状态、执行状态机控制、发送关节命令等核心控制逻辑，周期由 YAML 配置中的 "dt" 参数定义
    *  强化学习循环 loop_rl：负责调用 RL 模型进行推理，生成动作指令，周期为控制循环的整数倍（由 "decimation" 参数定义），以实现更高效的 RL 推理频率
    */
    if (this->robot_name != "bpx")
    {
        this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
        this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
        this->loop_control->start();
        this->loop_rl->start();
    
        // keyboard
        this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::PollKeyboard, this));
        this->loop_keyboard->start();
    
        // joystick
        this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
        this->loop_joystick->start();
    }

#ifdef PLOT
    if (this->robot_name == "bpx")
        throw std::runtime_error("BPX physics-owned control requires PLOT disabled");
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;

    // start simulation UI loop (blocking call)
    sim->RenderLoop();
    this->StopBpxPhysics();
    }
    catch (...)
    {
        this->StopBpxPhysics();
        instance = nullptr;
        throw;
    }
}

RL_Sim::~RL_Sim()
{
    this->StopBpxPhysics();
    instance = nullptr;
    if (loop_keyboard) loop_keyboard->shutdown();
    if (loop_joystick) loop_joystick->shutdown();
    if (loop_control) loop_control->shutdown();
    if (loop_rl) loop_rl->shutdown();
#ifdef PLOT
    if (loop_plot) loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StopBpxPhysics()
{
    if (!this->bpx_physics_thread_.joinable()) return;
    sim->exitrequest.store(1);
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
        this->bpx_ready_ = false;
        sim->cond_loadrequest.notify_all();
    }
    // Never join while holding sim->mtx: the physics thread may need it to exit.
    this->bpx_physics_thread_.join();
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
        // RenderLoop has returned (or never started). BPX worker deliberately
        // leaves its last model/data alive until the viewer and hook are done.
        sim->m_ = nullptr;
        sim->d_ = nullptr;
        sim->mnew_ = nullptr;
        sim->dnew_ = nullptr;
        sim->loadrequest = 0;
        mj_deleteData(d);
        mj_deleteModel(m);
        d = nullptr;
        m = nullptr;
    }
    this->mj_model = nullptr;
    this->mj_data = nullptr;
}

/**
 * @brief 从 MuJoCo 底层 mj_data 提取标准机器人状态 (IMU 姿态与电机反馈)。
 * @param state 传出参数，用于填充当前状态。
 * @note 调用方需确保处于临界区 (已加锁)，以避免在物理更新时产生脏读。
 */
void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            /*
            * state->motor_state.q[i]：关节位置
            * state->motor_state.dq[i]：关节速度
            * state->motor_state.tau_est[i]：关节力矩估计
            */
            if (this->robot_name == "bpx")
            {
                const int motor = this->params.Get<std::vector<int>>("joint_mapping")[i];
                const int joint = mj_model->actuator_trnid[2 * motor];
                state->motor_state.q[i] = mj_data->qpos[mj_model->jnt_qposadr[joint]];
                state->motor_state.dq[i] = mj_data->qvel[mj_model->jnt_dofadr[joint]];
                state->motor_state.tau_est[i] = mj_data->sensordata[motor + 24];
                continue;
            }
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
        }
    }
}

/**
 * @brief 执行底层的 PD 控制律，并将力矩计算结果写入 MuJoCo 的控制流。
 * @param command 包含期望的关节位置、速度及相应的阻抗/刚度增益参数。
 */
void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            if (this->robot_name == "bpx")
            {
                const int motor = this->params.Get<std::vector<int>>("joint_mapping")[i];
                const int joint = mj_model->actuator_trnid[2 * motor];
                const double q = mj_data->qpos[mj_model->jnt_qposadr[joint]];
                const double dq = mj_data->qvel[mj_model->jnt_dofadr[joint]];
                const double tau = command->motor_command.tau[i] +
                    command->motor_command.kp[i] * (command->motor_command.q[i] - q) +
                    command->motor_command.kd[i] * (command->motor_command.dq[i] - dq);
                if (!std::isfinite(tau)) throw std::runtime_error("BPX nonfinite PD torque");
                mj_data->ctrl[motor] = std::clamp(tau,
                    mj_model->actuator_ctrlrange[2 * motor],
                    mj_model->actuator_ctrlrange[2 * motor + 1]);
                continue;
            }
            // 核心执行逻辑：混合前馈力矩与反馈误差校正
            mj_data->ctrl[this->params.Get<std::vector<int>>("joint_mapping")[i]] =
                command->motor_command.tau[i] +
                command->motor_command.kp[i] * (command->motor_command.q[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]]) +
                command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")]);
        }
    }
}

/**
 * @brief 状态控制器，根据当前状态执行状态机控制逻辑。
 * @param state 当前机器人状态。
 * @param command 输出的机器人命令。
 */
void RL_Sim::RobotControl()
{
    if (simulation_exit_requested)
    {
        sim->exitrequest.store(1);
        return;
    }
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);

    if (this->robot_name == "bpx")
    {
        // Called exactly once per physical step; critical input was handled first.
        this->ClampBpxCommands();
        this->ClearBpxInput();
        return; // BpxPhysicsStep applies PD after possible same-step inference.
    }

    // Preserve the Go2 controller and wall-clock loop path.
    if (control.current_keyboard == Input::Keyboard::R || control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (mj_model && mj_data)
        {
            mj_resetData(mj_model, mj_data);
            mj_forward(mj_model, mj_data);
        }
    }
    if (control.current_keyboard == Input::Keyboard::Enter || control.current_gamepad == Input::Gamepad::RB_X)
    {
        sim->run = simulation_running ? 0 : 1;
        simulation_running = !simulation_running;
        std::cout << std::endl << LOGGER::INFO
                  << (simulation_running ? "Simulation Start" : "Simulation Stop") << std::endl;
    }
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
}

/**
 * @brief 初始化操纵杆文件描述符，并计算 ADC 采样归一化基准。
 */
void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

/**
 * @brief 操纵杆事件轮询回调。负责解析底层 input_event 并映射为高层指令。
 */
void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    // 重置瞬态按键事件位图，防止粘键
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    // BPX Passive wins over other button combinations sampled in this callback.
    if (this->robot_name == "bpx" && this->sys_js_button[4].pressed &&
        this->sys_js_button[2].on_press)
        this->control.SetGamepad(Input::Gamepad::LB_X);

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }
}

/**
 * @brief 低频模型推理任务。组装观测向量，执行 NN 推理，并将动作目标推入缓冲队列。
 */
void RL_Sim::RunModel()
{
    try
    {
        if (this->rl_init_done && simulation_running)
        {
            if (this->robot_name == "bpx") this->ClampBpxCommands();
            this->episode_length_buf += 1;
            // 构建 MDP (马尔可夫决策过程) 的状态向量 (Observation)
            this->obs.ang_vel = this->robot_state.imu.gyroscope;
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
            //not currently available for non-ros mujoco version
            // if (this->control.navigation_mode)
            // {
            //     this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
            // }
            this->obs.base_quat = this->robot_state.imu.quaternion;
            this->obs.dof_pos = this->robot_state.motor_state.q;
            this->obs.dof_vel = this->robot_state.motor_state.dq;

            this->obs.actions = this->Forward();
            // 将无量纲的网络输出转换为具体的物理控制参数
            this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

            // 基于生产者-消费者模式，将推理结果推入线程安全的队列，供高频控制回路异步拉取
            if (!this->output_dof_pos.empty())
            {
                output_dof_pos_queue.push(this->output_dof_pos);
            }
            if (!this->output_dof_vel.empty())
            {
                output_dof_vel_queue.push(this->output_dof_vel);
            }
            // BPX uses position/velocity queues and simulator-side PD. Its unused tau
            // queue must not grow for the duration of the simulation.
            if (this->robot_name != "bpx" && !this->output_dof_tau.empty())
            {
                output_dof_tau_queue.push(this->output_dof_tau);
            }

            // this->TorqueProtect(this->output_dof_tau);
            // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

    #ifdef CSV_LOGGER
            std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
            for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
            {
                tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
            }
            this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
    #endif
        }
    }
    catch (const std::exception& error)
    {
        if (this->robot_name != "bpx") throw;
        bpx_fsm::StopPolicyAndDrain(*this);
        this->InitControl();
        bpx_fsm::Damping(*this, &this->robot_state, &this->robot_command);
        this->fsm.RequestStateChange("RLFSMStatePassive");
        std::cerr << LOGGER::ERROR << "[BPX] Inference stopped: " << error.what() << std::endl;
    }
}

/**
 * @brief 带有线程安全锁和硬限幅保护的神经网络前向传播封装。
 * @return 预测的 Actions。若模型当前不可用（如正在热重载），则返回上一帧缓存以确保安全。
 */
std::vector<float> RL_Sim::Forward()
{
    // 采用 try_to_lock 获取非阻塞锁。防止在异步更新模型权重时阻塞核心计算流
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();
    if (this->robot_name == "bpx" && (clamped_obs.size() != 45 ||
        !std::all_of(clamped_obs.begin(), clamped_obs.end(), [](float value) { return std::isfinite(value); })))
        throw std::runtime_error("BPX actor requires 45 finite observations");


    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (this->robot_name == "bpx" && (actions.size() != 12 ||
        !std::all_of(actions.begin(), actions.end(), [](float value) { return std::isfinite(value); })))
        throw std::runtime_error("BPX actor requires 12 finite actions");

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}


void RL_Sim::PollKeyboard()
{
    if (this->robot_name == "bpx" &&
        (!::isatty(STDIN_FILENO) || ::tcgetpgrp(STDIN_FILENO) != ::getpgrp())) return;
    this->KeyboardInterface();
}

void RL_Sim::ClampBpxCommands()
{
    const auto limits = this->params.Get<std::vector<float>>("command_limits");
    if (limits.size() != 3)
        throw std::runtime_error("BPX command_limits requires three entries");
    this->control.x = std::clamp(this->control.x, -limits[0], limits[0]);
    this->control.y = std::clamp(this->control.y, -limits[1], limits[1]);
    this->control.yaw = std::clamp(this->control.yaw, -limits[2], limits[2]);
}

void RL_Sim::ValidateBpxModel()
{
    if (!this->mj_model || !this->mj_data || this->mj_model->nu != 12 ||
        this->mj_model->nq != 19 || this->mj_model->nv != 18 ||
        this->params.Get<int>("num_of_dofs") != 12)
        throw std::runtime_error("BPX expects a floating-base model with 12 motors");
    const auto names = this->params.Get<std::vector<std::string>>("joint_names");
    const auto mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const auto command_limits = this->params.Get<std::vector<float>>("command_limits");
    if (names.size() != 12 || mapping.size() != 12 || command_limits.size() != 3)
        throw std::runtime_error("BPX invalid joint/command configuration");
    for (float limit : command_limits)
        if (!std::isfinite(limit) || limit <= 0)
            throw std::runtime_error("BPX invalid command limit");
    std::vector<bool> used(12, false);
    const auto require_sensor = [&](const std::string& name, int address, int dim, int type) {
        const int id = mj_name2id(this->mj_model, mjOBJ_SENSOR, name.c_str());
        if (id < 0 || this->mj_model->sensor_adr[id] != address ||
            this->mj_model->sensor_dim[id] != dim || this->mj_model->sensor_type[id] != type)
            throw std::runtime_error("BPX incompatible sensor layout: " + name);
    };
    for (int i = 0; i < 12; ++i)
    {
        const int index = mapping[i];
        const int joint = mj_name2id(this->mj_model, mjOBJ_JOINT, names[i].c_str());
        if (index < 0 || index >= 12 || used[index] || joint < 0 ||
            this->mj_model->actuator_trntype[index] != mjTRN_JOINT ||
            this->mj_model->actuator_trnid[2 * index] != joint)
            throw std::runtime_error("BPX joint_mapping does not match its motors");
        used[index] = true;
        require_sensor(names[i] + "_pos", index, 1, mjSENS_JOINTPOS);
        require_sensor(names[i] + "_vel", 12 + index, 1, mjSENS_JOINTVEL);
        require_sensor(names[i] + "_tau", 24 + index, 1, mjSENS_JOINTACTFRC);
        if (!this->mj_model->actuator_ctrllimited[index] ||
            std::fabs(this->mj_model->actuator_ctrlrange[2 * index] + 30.0) > 1e-6 ||
            std::fabs(this->mj_model->actuator_ctrlrange[2 * index + 1] - 30.0) > 1e-6 ||
            std::fabs(this->mj_model->actuator_gear[6 * index] - 1.0) > 1e-6)
            throw std::runtime_error("BPX expects torque motors limited to +/-30 Nm");
    }
    require_sensor("body_quat", 36, 4, mjSENS_FRAMEQUAT);
    require_sensor("body_gyro", 40, 3, mjSENS_GYRO);
    const double dt = this->params.Get<float>("dt");
    if (!std::isfinite(dt) || std::fabs(dt - 0.005) > 1e-8 || this->params.Get<int>("decimation") != 4 ||
        std::fabs(this->mj_model->opt.timestep - dt) > 1e-8)
        throw std::runtime_error("BPX model timestep must match dt=0.005/decimation=4");
    if (this->mj_model->opt.disableflags & mjDSBL_CLAMPCTRL)
        throw std::runtime_error("BPX requires MuJoCo control clamping");
    if (mj_name2id(this->mj_model, mjOBJ_KEY, "home") < 0)
        throw std::runtime_error("BPX requires a home keyframe");
    // Report numerical failures instead of silently returning to qpos0.
    this->mj_model->opt.disableflags |= mjDSBL_AUTORESET;
}

void RL_Sim::ResetBpxSimulation(bool reset_fsm)
{
    // Only the BPX physics owner calls this after startup; caller holds sim->mtx.
    const int home = mj_name2id(this->mj_model, mjOBJ_KEY, "home");
    if (home < 0) throw std::runtime_error("BPX missing home keyframe");
    bpx_fsm::StopPolicyAndDrain(*this);
    mj_resetDataKeyframe(this->mj_model, this->mj_data, home);
    mj_forward(this->mj_model, this->mj_data);
    this->GetState(&this->robot_state);
    this->InitControl();
    this->obs.actions.assign(12, 0.0f);
    this->episode_length_buf = 0;
    this->bpx_policy_phase_ = 0;
    this->bpx_last_step_time_ = this->mj_data->time;
    this->bpx_faulted_ = false;
    this->control.current_keyboard = this->control.last_keyboard = Input::Keyboard::None;
    this->control.current_gamepad = this->control.last_gamepad = Input::Gamepad::None;
    if (reset_fsm)
    {
        const auto previous = this->fsm.current_state_;
        if (previous) previous->Exit();
        this->fsm.previous_state_ = previous;
        this->fsm.SetInitialState("RLFSMStatePassive");
        this->fsm.mode_ = FSM::Mode::NORMAL;
    }
    bpx_fsm::Damping(*this, &this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}


void RL_Sim::ClearBpxInput()
{
    control.current_keyboard = control.last_keyboard = Input::Keyboard::None;
    control.current_gamepad = control.last_gamepad = Input::Gamepad::None;
}

void RL_Sim::ServiceBpxInputs()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - bpx_keyboard_poll_ >= std::chrono::milliseconds(50))
    {
        PollKeyboard();
        bpx_keyboard_poll_ = now;
    }
    if (now - bpx_joystick_poll_ >= std::chrono::milliseconds(10))
    {
        GetSysJoystick();
        bpx_joystick_poll_ = now;
    }
    simulation_running = (sim->run != 0); // GUI pause is also authoritative.
    GetState(&robot_state);

    // Passive cancels a queued GetUp/RL transition, including while paused.
    if (bpx_fsm::WantsPassive(*this))
    {
        bpx_fsm::StopPolicyAndDrain(*this);
        const auto previous = fsm.current_state_;
        if (previous && previous->GetStateName() != "RLFSMStatePassive") previous->Exit();
        fsm.previous_state_ = previous;
        fsm.SetInitialState("RLFSMStatePassive");
        fsm.mode_ = FSM::Mode::NORMAL;
        InitControl();
        bpx_fsm::Damping(*this, &robot_state, &robot_command);
        bpx_policy_phase_ = 0;
        ClearBpxInput();
        return;
    }
    if (control.current_keyboard == Input::Keyboard::R || control.current_gamepad == Input::Gamepad::RB_Y)
    {
        ValidateBpxModel();
        ResetBpxSimulation(true);
        bpx_skip_next_step_ = true; // A reset is not a physical/control tick.
        return;
    }
    if (control.current_keyboard == Input::Keyboard::Enter || control.current_gamepad == Input::Gamepad::RB_X)
    {
        sim->run = !sim->run;
        simulation_running = (sim->run != 0);
        sim->speed_changed = true;
        ClearBpxInput();
        std::cout << std::endl << LOGGER::INFO
                  << (simulation_running ? "Simulation Start" : "Simulation Stop") << std::endl;
    }
    if (!sim->run)
    {
        // Paused 0/A prepares the transition; Enter later starts its trajectory.
        // Do not call FSM::Run(), GetUp::Enter() or inference while paused.
        if (fsm.current_state_ && fsm.current_state_->GetStateName() == "RLFSMStatePassive" &&
            (control.current_keyboard == Input::Keyboard::Num0 || control.current_gamepad == Input::Gamepad::A))
            fsm.RequestStateChange("RLFSMStateGetUp");
        ClearBpxInput();
    }
}

bool RL_Sim::BpxPhysicsStep(mjModel* model, mjData* data, bool advance)
{
    // PhysicsLoop holds sim->mtx. No independent BPX callbacks may touch RL state.
    if (simulation_exit_requested) sim->exitrequest.store(1);
    if (!bpx_ready_ || sim->exitrequest.load()) return false;
    try
    {
        if (mj_model != model || mj_data != data)
        {
            // The viewer loaded another model/data allocation. Never use old pointers.
            mj_model = model;
            mj_data = data;
            ValidateBpxModel();
            ResetBpxSimulation(true);
            bpx_skip_next_step_ = true;
        }
        // A rejected reload must never reach BPX sensor/mapping reads on later calls.
        // R can recover a control fault on a valid model; invalid models require reload.
        if (bpx_faulted_) ValidateBpxModel();
        if (data->time < bpx_last_step_time_)
        {
            ResetBpxSimulation(true);
            bpx_skip_next_step_ = true;
        }
        ServiceBpxInputs(); // Also services P/R/Enter during paused outer iterations.
        if (!advance) return false;
        if (bpx_skip_next_step_)
        {
            bpx_skip_next_step_ = false;
            return false;
        }
        if (bpx_faulted_) sim->run = 0;
        simulation_running = (sim->run != 0);
        if (!simulation_running) return false;

        // Update current IMU/derived quantities before taking this step's observation.
        // FSM/actor are not installed as mjcb_control, so forward cannot tick them.
        mj_forward(model, data);
        GetState(&robot_state);
        RobotControl(); // Exactly one FSM/trajectory tick per physical step.

        const auto policy_active = [this]() {
            return rl_init_done && fsm.mode_ == FSM::Mode::NORMAL && fsm.current_state_ &&
                   fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion";
        };
        if (policy_active())
        {
            if (bpx_policy_phase_ == 0)
            {
                RunModel();
                if (policy_active())
                {
                    // Consume the newly inferred target in THIS physical step.
                    // Calling FSM::Run() again here would double-tick the state machine.
                    auto state = std::dynamic_pointer_cast<RLFSMState>(fsm.current_state_);
                    if (!state) throw std::runtime_error("BPX policy state type mismatch");
                    state->RLControl();
                }
            }
        }
        else
        {
            bpx_fsm::StopPolicyAndDrain(*this);
            bpx_policy_phase_ = 0;
        }

        // Hold the position target for four steps, but recompute PD EVERY step.
        SetCommand(&robot_command);
        sim->InjectNoise();
        for (int motor = 0; motor < model->nu; ++motor)
        {
            if (!std::isfinite(data->ctrl[motor]))
                throw std::runtime_error("BPX nonfinite motor control");
            data->ctrl[motor] = std::clamp(data->ctrl[motor],
                model->actuator_ctrlrange[2 * motor], model->actuator_ctrlrange[2 * motor + 1]);
        }
        const double before = data->time;
        mj_step(model, data); // The only normal-running BPX integration site.
        for (mjtWarning warning : {mjWARN_BADQPOS, mjWARN_BADQVEL, mjWARN_BADQACC})
            if (data->warning[warning].number > 0)
                throw std::runtime_error(mju_warningText(warning, data->warning[warning].lastinfo));
        if (data->time <= before)
            throw std::runtime_error("BPX physics reset unexpectedly; inspect numerical warnings");
        bpx_last_step_time_ = data->time;
        bpx_policy_phase_ = policy_active() ?
            (bpx_policy_phase_ + 1) % params.Get<int>("decimation") : 0;
        return true;
    }
    catch (const std::exception& error)
    {
        sim->run = 0;
        simulation_running = false;
        sim->speed_changed = true;
        const bool first_fault = !bpx_faulted_;
        bpx_faulted_ = true;
        bpx_policy_phase_ = 0;
        bpx_fsm::StopPolicyAndDrain(*this);
        if (data) mju_zero(data->ctrl, model->nu);
        if (first_fault)
            std::cerr << LOGGER::ERROR << "[BPX] Simulation paused: " << error.what()
                      << "; fix the input/model, then use terminal R." << std::endl;
        return false;
    }
}

// Signal handler for Ctrl+C
void signalHandler(int)
{
    simulation_exit_requested = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    try
    {
        RL_Sim rl_sar(argc, argv);
        return 0;
    }
    catch (const std::exception& error)
    {
        RL_Sim::instance = nullptr;
        std::cerr << LOGGER::ERROR << error.what() << std::endl;
        return 1;
    }
}