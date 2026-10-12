/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RL_REAL_BPX_HPP
#define RL_REAL_BPX_HPP

#include "rl_sdk.hpp"
#include "fsm_bpx.hpp"
#include "joint_level_control.h"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <termios.h>

// RL/FSM and the SDK are owned by one periodic worker. The SDK retains its
// own network threads; no application thread concurrently mutates the RL core.
class RL_Real_Bpx final : public RL
{
public:
    explicit RL_Real_Bpx(const char* robot_ip);
    ~RL_Real_Bpx();
    bool IsRunning() const noexcept { return !stop_requested_.load(); }
    bool HasFault() const noexcept { return faulted_.load(); }

private:
    using Clock = std::chrono::steady_clock;
    struct Snapshot
    {
        std::array<float, 12> q{}, dq{}, tau{};
        std::array<float, 4> quat{}; // SDK x,y,z,w
        std::array<float, 3> omega{}, acc{};
        uint32_t seq = 0;
    };

    std::vector<float> Forward() override;
    void GetState(RobotState<float>* state) override;
    void SetCommand(const RobotCommand<float>* command) override;
    void LoadConfiguration();
    void PreloadPolicy();
    void ConnectAndWait(const char* robot_ip);
    bool ReadSnapshot(Snapshot& snapshot);
    void RobotControl();
    void RunModel();
    bool PolicyActive() const;
    void ForcePassive();
    void ResetInputEvents();
    void SetupTerminal();
    void RestoreTerminal() noexcept;
    void PollKeyboard();
    void StartLoop();
    void StopLoop() noexcept;
    void ControlLoop() noexcept;
    void BestEffortDamping() noexcept;
    void Close() noexcept;

    bpx_sdk::JointLevelControl sdk_;
    YAML::Node base_config_;
    std::array<int, 12> mapping_{};
    std::array<float, 12> sign_{}, offset_{};
    std::array<float, 4> body_from_imu_{}; // wxyz; rotates IMU vectors into torso
    std::array<float, 3> velocity_limits_{};
    float gyro_scale_ = 1.0f;
    bool omega_in_world_ = false;
    int state_timeout_ms_ = 0, connect_timeout_ms_ = 0;
    int period_ms_ = 5, decimation_ = 4, policy_phase_ = 0;
    Snapshot snapshot_{};
    bool have_snapshot_ = false, command_started_ = false;
    Clock::time_point last_new_state_{};
    termios original_terminal_{};
    bool terminal_changed_ = false;
    std::atomic<bool> stop_requested_{false}, faulted_{false};
    std::mutex stop_mutex_;
    std::condition_variable stop_cv_;
    std::thread worker_;
};
#endif // RL_REAL_BPX_HPP