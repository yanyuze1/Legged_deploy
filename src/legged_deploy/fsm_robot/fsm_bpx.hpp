/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BPX_FSM_HPP
#define BPX_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

// Shared BPX FSM for MuJoCo and the BPX real entry.
// The real entry preloads the model and serializes FSM/inference/queue access.
namespace bpx_fsm
{
inline void StopPolicyAndDrain(RL& rl)
{
    rl.rl_init_done = false;
    std::vector<float> discarded;
    while (rl.output_dof_pos_queue.try_pop(discarded)) {}
    while (rl.output_dof_vel_queue.try_pop(discarded)) {}
    while (rl.output_dof_tau_queue.try_pop(discarded)) {}
}

inline void RequireJointVector(
    const std::vector<float>& values,
    const char* description)
{
    if (values.size() != 12)
        throw std::runtime_error(std::string("BPX expects twelve entries in ") + description);
    for (float value : values)
        if (!std::isfinite(value))
            throw std::runtime_error(std::string("BPX nonfinite value in ") + description);
}

inline std::vector<float> RequireVector(RL& rl, const char* key)
{
    if (rl.params.Get<int>("num_of_dofs") != 12)
        throw std::runtime_error("BPX expects twelve joints");
    auto values = rl.params.Get<std::vector<float>>(key);
    RequireJointVector(values, key);
    return values;
}

inline void RequireFixedGains(RL& rl)
{
    const auto kp = RequireVector(rl, "fixed_kp");
    const auto kd = RequireVector(rl, "fixed_kd");
    for (size_t i = 0; i < 12; ++i)
        if (kp[i] < 0.0f || kd[i] < 0.0f)
            throw std::runtime_error("BPX negative fixed PD gain");
}

inline float Duration(RL& rl, const char* key, float fallback)
{
    const float value = rl.params.Get<float>(key, fallback);
    const float dt = rl.params.Get<float>("dt");
    const double frames = std::ceil(value / dt);
    if (!std::isfinite(value) || value <= 0.0f ||
        !std::isfinite(dt) || dt <= 0.0f ||
        !std::isfinite(frames) || frames < 1.0 ||
        frames > std::numeric_limits<int>::max())
        throw std::runtime_error(std::string("BPX invalid interpolation duration or dt: ") + key);
    return value;
}

inline bool WantsPassive(const RL& rl)
{
    return rl.control.current_keyboard == Input::Keyboard::P ||
           rl.control.current_gamepad == Input::Gamepad::LB_X;
}

inline void Damping(
    RL& rl,
    const RobotState<float>* state,
    RobotCommand<float>* command)
{
    const float kd = rl.params.Get<float>("passive_kd", 1.0f);
    if (!std::isfinite(kd) || kd < 0.0f || rl.params.Get<int>("num_of_dofs") != 12)
        throw std::runtime_error("BPX invalid Passive gain or joint count");
    RequireJointVector(state->motor_state.q, "measured position");
    for (int i = 0; i < 12; ++i)
    {
        command->motor_command.q[i] = state->motor_state.q[i];
        command->motor_command.dq[i] = 0.0f;
        command->motor_command.kp[i] = 0.0f;
        command->motor_command.kd[i] = kd;
        command->motor_command.tau[i] = 0.0f;
    }
}

inline void SeedStandingCommand(RL& rl, RobotCommand<float>* command)
{
    const auto pose = RequireVector(rl, "default_dof_pos");
    const auto kp = RequireVector(rl, "rl_kp");
    const auto kd = RequireVector(rl, "rl_kd");
    for (int i = 0; i < 12; ++i)
    {
        if (kp[i] < 0.0f || kd[i] < 0.0f)
            throw std::runtime_error("BPX negative policy PD gain");
        command->motor_command.q[i] = pose[i];
        command->motor_command.dq[i] = 0.0f;
        command->motor_command.kp[i] = kp[i];
        command->motor_command.kd[i] = kd[i];
        command->motor_command.tau[i] = 0.0f;
    }
}

class RLFSMStatePassive : public RLFSMState
{
public:
    explicit RLFSMStatePassive(RL* context)
        : RLFSMState(*context, "RLFSMStatePassive") {}

    void Enter() override
    {
        StopPolicyAndDrain(rl);
        rl.InitControl();
        std::cout << LOGGER::NOTE
                  << "[BPX] Passive; press 0/A to stand from the current pose."
                  << std::endl;
    }

    void Run() override
    {
        Damping(rl, fsm_state, fsm_command);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (WantsPassive(rl)) return state_name_;
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    explicit RLFSMStateGetUp(RL* context)
        : RLFSMState(*context, "RLFSMStateGetUp") {}

    void Enter() override
    {
        StopPolicyAndDrain(rl);
        rl.InitControl();
        percent_ = 0.0f;
        RequireJointVector(fsm_state->motor_state.q, "GetUp measured position");
        rl.now_state = *fsm_state;
        start_ = rl.now_state.motor_state.q;
        target_ = RequireVector(rl, "default_dof_pos");
        RequireFixedGains(rl);
        duration_ = Duration(rl, "getup_duration", 2.0f);

        // Keep the original resting pose through RL -> GetUp recovery.
        // A new Passive -> GetUp transition begins a new standing cycle.
        const bool from_passive =
            rl.fsm.previous_state_ &&
            rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive";
        if (from_passive || !has_saved_start_)
        {
            rl.start_state = rl.now_state;
            has_saved_start_ = true;
        }
    }

    void Run() override
    {
        // Use the same interpolation helper as Go2. In particular, a motion
        // below its existing 0.1-rad threshold completes immediately.
        Interpolate(percent_, start_, target_, duration_, "[BPX] Getting up", true);
    }

    void Exit() override {}

    const std::vector<float>& StartPosition() const noexcept { return start_; }
    bool HasSavedStart() const noexcept { return has_saved_start_; }
    bool TrajectoryComplete() const noexcept { return percent_ >= 1.0f; }
    float DurationSeconds() const noexcept { return duration_; }

    std::string CheckChange() override
    {
        if (WantsPassive(rl)) return "RLFSMStatePassive";
        if (TrajectoryComplete())
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
                rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
                return "RLFSMStateRLLocomotion";
            if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
                rl.control.current_gamepad == Input::Gamepad::B)
                return "RLFSMStateGetDown";
        }
        return state_name_;
    }

private:
    bool has_saved_start_ = false;
    float percent_ = 0.0f;
    float duration_ = 2.0f;
    std::vector<float> start_, target_;
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    explicit RLFSMStateGetDown(RL* context)
        : RLFSMState(*context, "RLFSMStateGetDown") {}

    void Enter() override
    {
        StopPolicyAndDrain(rl);
        rl.InitControl();
        percent_ = 0.0f;
        RequireJointVector(fsm_state->motor_state.q, "GetDown measured position");
        const auto found = rl.fsm.states_.find("RLFSMStateGetUp");
        const auto getup = found == rl.fsm.states_.end()
            ? nullptr
            : std::dynamic_pointer_cast<RLFSMStateGetUp>(found->second);
        if (!getup || !getup->HasSavedStart())
            throw std::runtime_error("BPX GetDown requires a captured resting position");
        RequireJointVector(rl.start_state.motor_state.q, "saved resting position");
        rl.now_state = *fsm_state;
        start_ = rl.now_state.motor_state.q;
        target_ = rl.start_state.motor_state.q;
        RequireFixedGains(rl);
        duration_ = Duration(rl, "getdown_duration", 2.0f);
    }

    void Run() override
    {
        Interpolate(percent_, start_, target_, duration_, "[BPX] Getting down", true);
    }

    void Exit() override {}

    bool TrajectoryComplete() const noexcept { return percent_ >= 1.0f; }

    std::string CheckChange() override
    {
        if (WantsPassive(rl) || TrajectoryComplete())
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        return state_name_;
    }

private:
    float percent_ = 0.0f;
    float duration_ = 2.0f;
    std::vector<float> start_, target_;
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    explicit RLFSMStateRLLocomotion(RL* context)
        : RLFSMState(*context, "RLFSMStateRLLocomotion") {}

    void Enter() override
    {
        loaded_ = false;
        StopPolicyAndDrain(rl);
        rl.config_name = "mirrorme";
        rl.episode_length_buf = 0;
        try
        {
#ifdef USE_BPX_REAL
            // The BPX real entry preloads and warms the model before its 5-ms
            // worker starts, so RL entry does not perform disk loading.
            if (!rl.model || !rl.model->is_loaded())
                throw std::runtime_error("BPX real policy must be preloaded");
            rl.InitObservations();
            rl.InitOutputs();
            rl.InitControl();
#else
            const std::string profile = "bpx/mirrorme";
            const auto yaml = YAML::LoadFile(
                std::string(POLICY_DIR) + "/" + profile + "/config.yaml");
            if (!yaml[profile] || !yaml[profile].IsMap())
                throw std::runtime_error("Missing YAML mapping bpx/mirrorme");
            rl.InitRL(profile);
#endif
            if (rl.params.Get<int>("num_of_dofs") != 12 ||
                rl.params.Get<int>("num_observations") != 45 ||
                !rl.params.Get<std::vector<int>>("observations_history").empty())
                throw std::runtime_error(
                    "BPX actor requires twelve joints and one 45-float frame without history");

            rl.obs.ang_vel = fsm_state->imu.gyroscope;
            rl.obs.base_quat = fsm_state->imu.quaternion;
            rl.obs.dof_pos = fsm_state->motor_state.q;
            rl.obs.dof_vel = fsm_state->motor_state.dq;
            rl.obs.actions.assign(12, 0.0f);
            rl.obs.commands = {0.0f, 0.0f, 0.0f};
            SeedStandingCommand(rl, fsm_command);
            loaded_ = true;
        }
        catch (const std::exception& error)
        {
            StopPolicyAndDrain(rl);
            Damping(rl, fsm_state, fsm_command);
            std::cerr << LOGGER::ERROR
                      << "[BPX] Policy setup failed: "
                      << error.what() << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!loaded_)
        {
            rl.rl_init_done = false;
            Damping(rl, fsm_state, fsm_command);
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        rl.rl_init_done = true;
#ifdef USE_BPX_REAL
        const char* limits_key = "real_command_limits";
#else
        const char* limits_key = "command_limits";
#endif
        const auto limits = rl.params.Get<std::vector<float>>(limits_key);
        if (limits.size() != 3)
            throw std::runtime_error("BPX expects three velocity command limits");
        for (float limit : limits)
            if (!std::isfinite(limit) || limit < 0.0f)
                throw std::runtime_error("BPX invalid velocity command limit");
        if (!std::isfinite(rl.control.x) || !std::isfinite(rl.control.y) ||
            !std::isfinite(rl.control.yaw))
            throw std::runtime_error("BPX nonfinite velocity command");
        rl.control.x = std::clamp(rl.control.x, -limits[0], limits[0]);
        rl.control.y = std::clamp(rl.control.y, -limits[1], limits[1]);
        rl.control.yaw = std::clamp(rl.control.yaw, -limits[2], limits[2]);
        std::cout << "\r\033[K" << std::flush
                  << LOGGER::INFO
                  << "RL Controller [" << rl.config_name << "]"
                  << " x:" << rl.control.x
                  << " y:" << rl.control.y
                  << " yaw:" << rl.control.yaw
                  << std::flush;
        RLControl();
    }

    void Exit() override
    {
        loaded_ = false;
        StopPolicyAndDrain(rl);
        rl.InitControl();
    }

    std::string CheckChange() override
    {
        if (WantsPassive(rl)) return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
            return "RLFSMStateGetDown";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        return state_name_;
    }

private:
    bool loaded_ = false;
};
} // namespace bpx_fsm

class BpxFSMFactory : public FSMFactory
{
public:
    explicit BpxFSMFactory(const std::string& initial)
        : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(
        void* context,
        const std::string& state_name) override
    {
        RL* rl = static_cast<RL*>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<bpx_fsm::RLFSMStatePassive>(rl);
        if (state_name == "RLFSMStateGetUp")
            return std::make_shared<bpx_fsm::RLFSMStateGetUp>(rl);
        if (state_name == "RLFSMStateGetDown")
            return std::make_shared<bpx_fsm::RLFSMStateGetDown>(rl);
        if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<bpx_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }

    std::string GetType() const override { return "bpx"; }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }

    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(BpxFSMFactory, "RLFSMStatePassive")

#endif // BPX_FSM_HPP
