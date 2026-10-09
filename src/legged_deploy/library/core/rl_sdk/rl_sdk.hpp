/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
 * @file rl_sdk.hpp
 * @brief 强化学习控制 SDK 核心头文件。
 * 定义了硬件抽象数据结构、系统级输入枚举、并发通信队列以及 RL 控制器核心基类。
 */
#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <iostream>
#include <string>
#include <exception>
#include <unistd.h>
#include <algorithm>
#include <tbb/concurrent_queue.h>
#include <vector>
#include <memory>
#include <fstream>
#include <mutex>

#include <yaml-cpp/yaml.h>
#include "fsm.hpp"
#include "observation_buffer.hpp"
#include "vector_math.hpp"
#include "inference_runtime.hpp"
#include "logger.hpp"
#include "motion_loader.hpp"

/**
 * @brief 机器人底层控制指令模板结构。
 * @details 将算法层的期望输出映射到底层电机控制律。
 */
template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode;  // 电机控制模式
        std::vector<T> q;       // 目标电机角度
        std::vector<T> dq;      // 目标电机速度
        std::vector<T> tau;     // 目标电机力矩
        std::vector<T> kp;      // 目标电机位置刚度
        std::vector<T> kd;      // 目标电机速度阻尼

        void resize(size_t num_joints)
        // 按照机器人关节数量初始化电机命令数组
        {
            mode.resize(num_joints, 0);
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            tau.resize(num_joints, 0.0f);
            kp.resize(num_joints, 0.0f);
            kd.resize(num_joints, 0.0f);
        }
    } motor_command;
};

/**
 * @brief 机器人底层传感器状态模板结构。
 * @details 标准化真实硬件或仿真器回传的遥测数据 (Telemetry)。
 */
template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
        std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};        // 角速度
        std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};    // 线速度
    } imu;

    struct MotorState
    {
        std::vector<T> q;       // 实际关节角度
        std::vector<T> dq;      // 实际关节速度
        std::vector<T> ddq;     // 实际关节加速度
        std::vector<T> tau_est; // 电机力矩估计/反馈力矩
        std::vector<T> cur;     // 电机相电流

        void resize(size_t num_joints)
        {
            q.resize(num_joints, 0.0f);    
            dq.resize(num_joints, 0.0f);
            ddq.resize(num_joints, 0.0f);
            tau_est.resize(num_joints, 0.0f);
            cur.resize(num_joints, 0.0f);
        }
    } motor_state;
};

namespace Input
/**
 * @namespace Input
 * @brief 硬件外设输入语义映射。
 * @details 将键盘/手柄的物理按键映射为通用的系统级枚举，实现输入设备的解耦。
 */
{
    // Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
    //            R-SimReset Enter-SimToggle
    //            M-MotorEnable K-MotorDisable P-MotorPassive
    //            Num1-BaseLocomotion Num2-Num8-Skills(7)
    //            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
    enum class Keyboard
    {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Space, Enter, Escape,
        Up, Down, Left, Right
    };

    // Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
    //            RB_Y-SimReset RB_X-SimToggle
    //            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
    //            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
    //            LY-AxisX LX-AxisY RX-AxisYaw
    enum class Gamepad
    {
        None = 0,
        A, B, X, Y, LB, RB, LStick, RStick, DPadUp, DPadDown, DPadLeft, DPadRight,
        LB_A, LB_B, LB_X, LB_Y, LB_LStick, LB_RStick, LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
        RB_A, RB_B, RB_X, RB_Y, RB_LStick, RB_RStick, RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight,
        LB_RB
    };
}

struct Control
/**
 * @struct Control
 * @brief 高层控制指令集与状态缓冲。
 * @details 处理按键去抖、记录速度期望值，提供给神经网络或状态机消费。
 */
{
    Input::Keyboard current_keyboard = Input::Keyboard::None, last_keyboard = Input::Keyboard::None;
    Input::Gamepad current_gamepad = Input::Gamepad::None, last_gamepad = Input::Gamepad::None;

    float x = 0.0f;     // x轴线速度
    float y = 0.0f;     // y轴线速度
    float yaw = 0.0f;   // yaw角速度
    bool navigation_mode = false;

    void SetKeyboard(Input::Keyboard keyboad)
    {
        if (current_keyboard != keyboad)
        {
            last_keyboard = current_keyboard;
            current_keyboard = keyboad;
        }
    }

    void SetGamepad(Input::Gamepad gamepad)
    {
        if (current_gamepad != gamepad)
        {
            last_gamepad = current_gamepad;
            current_gamepad = gamepad;
        }
    }

    void ClearInput()
    {
        current_keyboard = last_keyboard;
        current_gamepad = Input::Gamepad::None;
    }
};

/**
 * @struct YamlParams
 * @brief YAML 配置文件安全包装器。
 */
struct YamlParams
{
    YAML::Node config_node;

    // Get config value by key
    // WARNING: For vectors/containers, store result in a variable before using iterators/references:
    //   ✓ auto vec = params.Get<std::vector<int>>("key"); vec.begin()
    //   ✗ params.Get<std::vector<int>>("key").begin()  // dangling reference!
    template<typename T>
    T Get(const std::string& key, const T& default_value = T()) const
    {
        if (config_node[key])
        {
            return config_node[key].as<T>();
        }
        return default_value;
    }

    bool Has(const std::string& key) const
    {
        return config_node[key].IsDefined();
    }
};

/**
 * @struct Observations
 * @brief 强化学习马尔可夫决策过程 (MDP) 的观测空间定义。
 */
template <typename T>
struct Observations
{
    std::vector<T> lin_vel;             // 基座线速度
    std::vector<T> ang_vel;             // 基座角速度
    std::vector<T> gravity_vec;         // 重力向量
    std::vector<T> commands;            // 速度指令向量
    std::vector<T> base_quat;           // 基座四元数
    std::vector<T> dof_pos;             // 关节位置/角度
    std::vector<T> dof_vel;             // 关节速度
    std::vector<T> actions;             // 动作向量
};

/**
 * @class RL
 * @brief 强化学习机器人控制器上下文核心基类。
 * @details 作为全局黑板 (Blackboard)，串联底层 HAL、状态机、多线程缓冲及 ONNX/LibTorch 推理引擎。
 */
class RL
{
public:
    RL() {};
    ~RL() {};

    YamlParams params;
    Observations<float> obs;
    std::vector<int> obs_dims;

    RobotState<float> robot_state;
    RobotCommand<float> robot_command;
    /* ===================================================================== */
    /* Thread-Safe Inter-Process Communication (生产者-消费者模型)          */
    /* ===================================================================== */
    // 低频 RL 线程作为 Producer 推入数据，高频 PD 控制线程作为 Consumer 拉取数据
    tbb::concurrent_queue<std::vector<float>> output_dof_pos_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_vel_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_tau_queue;

    FSM fsm;
    RobotState<float> start_state;
    RobotState<float> now_state;
    bool rl_init_done = false;          ///< RL 模型加载完毕就绪标志，用于解锁推理线程

    // init 初始化例程
    void InitObservations();
    void InitOutputs();
    void InitControl();
    void InitRL(std::string robot_config_path);
    void InitJointNum(size_t num_joints);

    // rl functions
    /* ===================================================================== */
    /* Core Abstract Interfaces (需由子类实现，例如 rl_sim_mujoco.cpp)     */
    /* ===================================================================== */
    virtual std::vector<float> Forward() = 0;
    std::vector<float> ComputeObservation();
    virtual void GetState(RobotState<float> *state) = 0;
    // 具体执行逻辑
    virtual void SetCommand(const RobotCommand<float> *command) = 0;
    void StateController(const RobotState<float> *state, RobotCommand<float> *command);
    void ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau);

    // yaml params
    void ReadYaml(const std::string& file_path, const std::string& file_name);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(const std::vector<float> &torque, const std::vector<float> &tau_est, const std::vector<float> &joint_pos, const std::vector<float> &joint_pos_target, const std::vector<float> &joint_vel);

    // control
    Control control;
    void KeyboardInterface();

    // history buffer
    ObservationBuffer history_obs_buf;
    std::vector<float> history_obs;

    // others
    int motiontime = 0;
    std::string robot_name, config_name;
    bool simulation_running = true;
    std::string ang_vel_axis = "body";  // "world" or "body"
    unsigned long long episode_length_buf = 0;
    float motion_length = 0.0;
    int InverseJointMapping(int idx) const;

    // Motion tracking (for mimic/dance tasks)
    std::unique_ptr<MotionLoader> motion_loader;

    // protect func
    void TorqueProtect(const std::vector<float> &origin_output_dof_tau);
    void AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold);

    // rl module
    std::unique_ptr<InferenceRuntime::Model> model;
    // output buffer
    std::vector<float> output_dof_tau;
    std::vector<float> output_dof_pos;
    std::vector<float> output_dof_vel;

    // thread safety
    std::mutex model_mutex;     //  模型重载/读写互斥锁，保障多线程环境下的推理线程安全
};

/**
 * @class RLFSMState
 * @brief 专属于 RL 控制流的状态机节点基类。
 * @details 携带对核心全局上下文 `RL&` 的引用，为特定状态提供插值工具与网络接管能力。
 */
class RLFSMState : public FSMState
{
public:
    RLFSMState(RL& rl, const std::string& name)
        : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}

    RL& rl;                       // RL 模型控制核心上下文
    const RobotState<float> *fsm_state;
    RobotCommand<float> *fsm_command;

    /**
     * @brief 关节角度平滑插值器。
     * @details 常用于起立/趴下等非学习任务中的固定轨迹生成，避免力矩阶跃。
     */
    bool Interpolate(
        float& percent,
        const std::vector<float>& start_pos,
        const std::vector<float>& target_pos,
        float duration_seconds,
        const std::string& description = "",
        bool use_fixed_gains = true
    );

    /** @brief 代理执行底层神经网络推理回调 */
    void RLControl();
};

#endif // RL_SDK_HPP
