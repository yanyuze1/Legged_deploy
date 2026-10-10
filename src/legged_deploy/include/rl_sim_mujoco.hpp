/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

 
#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_go2.hpp"
#include "fsm_bpx.hpp"
#include <mutex>
#include <thread>
#include <chrono>

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Button
{
public:
    Button() {}

    /**
     * @brief 更新按键状态，并计算瞬态边缘信号 (Rising/Falling Edge)
     * @param state 当前的硬件电平或布尔状态
     */
    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    std::unique_ptr<mj::Simulate> sim;
    static RL_Sim* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // BPX: one owner for input, FSM, inference and PD, under sim->mtx.
    // No BPX wall-clock control/model/input loops may run concurrently.
    std::thread bpx_physics_thread_;
    bool bpx_ready_ = false;
    bool bpx_skip_next_step_ = false;
    bool bpx_faulted_ = false;
    int bpx_policy_phase_ = 0;
    double bpx_last_step_time_ = 0.0;
    std::chrono::steady_clock::time_point bpx_keyboard_poll_{};
    std::chrono::steady_clock::time_point bpx_joystick_poll_{};
    bool BpxPhysicsStep(mjModel* model, mjData* data, bool advance);
    void ServiceBpxInputs();
    void ClearBpxInput();
    void StopBpxPhysics();
    void PollKeyboard();
    void ValidateBpxModel();
    void ResetBpxSimulation(bool reset_fsm);
    void ClampBpxCommands();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data = nullptr;
    mjModel *mj_model = nullptr;
    std::string scene_name;

    // joystick
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();

    // others 额外扩展接口
    std::string gazebo_model_name;
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);
};

#endif // RL_SIM_HPP