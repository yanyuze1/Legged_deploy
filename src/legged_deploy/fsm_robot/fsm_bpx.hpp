/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BPX_FSM_HPP
#define BPX_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <cmath>
#include <stdexcept>

// BPX simulation reference. This is not a real-robot controller.
// Scheduling requirement: the BPX RobotControl and RunModel callbacks must hold
// the same entry-local mutex for their entire callbacks. A false rl_init_done
// or the existing Forward-only model_mutex does not synchronize queue producers.
// Go2 can retain its existing path; do not change its FSM or observation contract.
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

inline std::vector<float> RequireVector(RL& rl, const char* key)
{
    auto values = rl.params.Get<std::vector<float>>(key);
    if (rl.params.Get<int>("num_of_dofs") != 12 || values.size() != 12)
        throw std::runtime_error(std::string("BPX expects twelve entries in ") + key);
    for (float value : values)
        if (!std::isfinite(value))
            throw std::runtime_error(std::string("BPX nonfinite value in ") + key);
    return values;
}

inline float Duration(RL& rl, const char* key, float fallback)
{
    const float value = rl.params.Get<float>(key, fallback);
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error(std::string("BPX invalid duration: ") + key);
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
    if (!std::isfinite(kd) || kd < 0 || rl.params.Get<int>("num_of_dofs") != 12)
        throw std::runtime_error("BPX invalid Passive gain or joint count");
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
        if (kp[i] < 0 || kd[i] < 0)
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
                  << "[BPX] Passive; press 0/A for nominal stand."
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
        start_ = fsm_state->motor_state.q;
        target_ = RequireVector(rl, "default_dof_pos");
        RequireVector(rl, "fixed_kp");
        RequireVector(rl, "fixed_kd");
        duration_ = Duration(rl, "getup_duration", 2.0f);
        // Policy/group order: roll(FL,FR,HL,HR), pitch(...), knee(...).
        // No Go2-specific tucked pre_running_pos is reused here.
    }

    void Run() override
    {
        Interpolate(
            percent_,
            start_,
            target_,
            duration_,
            "[BPX] Nominal stand",
            true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (WantsPassive(rl)) return "RLFSMStatePassive";
        if (percent_ >= 1.0f)
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
    float percent_ = 0.0f;
    float duration_ = 2.0f; // Deployment interpolation setting, not a training parameter.
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
        start_ = fsm_state->motor_state.q;
        target_ = RequireVector(rl, "default_dof_pos");
        RequireVector(rl, "fixed_kp");
        RequireVector(rl, "fixed_kd");
        duration_ = Duration(rl, "getdown_duration", 1.0f);
        std::cout << LOGGER::NOTE
                  << "[BPX] Return to nominal stand, then Passive; no lying-down trajectory."
                  << std::endl;
    }

    void Run() override
    {
        Interpolate(
            percent_,
            start_,
            target_,
            duration_,
            "[BPX] Return to stand",
            true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (WantsPassive(rl) || percent_ >= 1.0f)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        return state_name_;
    }

private:
    float percent_ = 0.0f;
    float duration_ = 1.0f;
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
            // ReadYaml indexes the full path key, not only the last component.
            const std::string profile = "bpx/mirrorme";
            const auto yaml = YAML::LoadFile(
                std::string(POLICY_DIR) + "/" + profile + "/config.yaml");
            if (!yaml[profile] || !yaml[profile].IsMap())
                throw std::runtime_error("Missing YAML mapping bpx/mirrorme");

            rl.InitRL(profile);

            if (rl.params.Get<int>("num_observations") != 45 ||
                !rl.params.Get<std::vector<int>>("observations_history").empty())
                throw std::runtime_error(
                    "BPX actor requires one 45-float frame without history");

            rl.obs.ang_vel = fsm_state->imu.gyroscope;
            rl.obs.base_quat = fsm_state->imu.quaternion;
            rl.obs.dof_pos = fsm_state->motor_state.q;
            rl.obs.dof_vel = fsm_state->motor_state.dq;
            rl.obs.actions.assign(12, 0.0f);
            rl.obs.commands = {0.0f, 0.0f, 0.0f};
            SeedStandingCommand(rl, fsm_command);
            loaded_ = true;
            // rl_init_done is enabled by Run only after every setup step succeeds.
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

        // Same velocity display behavior as the current Go2 FSM.
        std::cout << "\r\033[K" << std::flush
                  << LOGGER::INFO
                  << "RL Controller [" << rl.config_name << "]"
                  << " x:" << rl.control.x
                  << " y:" << rl.control.y
                  << " yaw:" << rl.control.yaw
                  << std::flush;

        // No new frame: retain the seeded stand or the previous policy command.
        // Do not reset to default_dof_pos on every 200-Hz control heartbeat.
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

    std::string GetType() const override
    {
        return "bpx";
    }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"
        };
    }

    std::string GetInitialState() const override
    {
        return initial_state_;
    }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(BpxFSMFactory, "RLFSMStatePassive")

#endif // BPX_FSM_HPP