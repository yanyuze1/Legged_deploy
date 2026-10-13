/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RL_REAL_BPX_HPP
#define RL_REAL_BPX_HPP

#include "rl_sdk.hpp"
#include "fsm_bpx.hpp"
#include "joint_level_control.h"
#include "joystick.hh"
#include <cerrno>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <termios.h>

namespace bpx_teleop
{
struct LocalGamepadSample
{
    bool connected = false;
    bool ready = false;
    bool passive = false;
    Input::Gamepad mode = Input::Gamepad::None;
    std::array<float, 3> axes{};
    // MuJoCo uses these normalized values directly, without keyboard scaling.
    std::array<float, 3> limits{1.0f, 1.0f, 1.0f};
};

struct LocalButton
{
    // Same rising/falling-edge calculation as RL_Sim's Button::update().
    void update(bool state) noexcept
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class SimJoystickDecoder
{
public:
    void BeginPoll() noexcept
    {
        mode_ = Input::Gamepad::None;
        initial_batch_ = false;
        passive_in_batch_ = false;
        for (auto& button : buttons_)
        {
            button.on_press = false;
            button.on_release = false;
        }
    }

    void Consume(const JoystickEvent& event) noexcept
    {
        const bool initial = (event.type & JS_EVENT_INIT) != 0;
        if (initial)
        {
            // Initial states synchronize levels but never authorize an action.
            initial_batch_ = true;
            RequireNeutral();
        }
        if ((event.type & JS_EVENT_BUTTON) != 0 && event.number < buttons_.size())
        {
            const auto number = static_cast<std::size_t>(event.number);
            buttons_seen_[number] = true;
            if (initial)
            {
                buttons_[number].pressed = event.value != 0;
                buttons_[number].on_press = false;
                buttons_[number].on_release = false;
            }
            else
                buttons_[number].update(event.value != 0);
        }
        else if ((event.type & JS_EVENT_AXIS) != 0 && event.number < axes_.size())
        {
            const auto number = static_cast<std::size_t>(event.number);
            axes_seen_[number] = true;
            const double normalized = static_cast<double>(event.value) / MaxValue;
            axes_[number] = std::abs(normalized) < Deadzone ? 0 : event.value;
        }
        if (!initial && was_ready_ && buttons_[4].pressed && buttons_[2].pressed)
            passive_in_batch_ = true;
    }

    void FinishPoll() noexcept
    {
        if (!ready_ && HaveRequiredStates() && IsNeutral())
        {
            ready_ = true;
            was_ready_ = true;
        }
        mode_ = Input::Gamepad::None;
        if (initial_batch_)
            return;
        if (ready_)
        {
            // Preserve GetSysJoystick's complete mapping and original order.
            if (buttons_[0].on_press) mode_ = Input::Gamepad::A;
            if (buttons_[1].on_press) mode_ = Input::Gamepad::B;
            if (buttons_[2].on_press) mode_ = Input::Gamepad::X;
            if (buttons_[3].on_press) mode_ = Input::Gamepad::Y;
            if (buttons_[4].on_press) mode_ = Input::Gamepad::LB;
            if (buttons_[5].on_press) mode_ = Input::Gamepad::RB;
            if (buttons_[9].on_press) mode_ = Input::Gamepad::LStick;
            if (buttons_[10].on_press) mode_ = Input::Gamepad::RStick;
            if (axes_[7] < 0) mode_ = Input::Gamepad::DPadUp;
            if (axes_[7] > 0) mode_ = Input::Gamepad::DPadDown;
            if (axes_[6] > 0) mode_ = Input::Gamepad::DPadLeft;
            if (axes_[6] < 0) mode_ = Input::Gamepad::DPadRight;
            if (buttons_[4].pressed && buttons_[0].on_press) mode_ = Input::Gamepad::LB_A;
            if (buttons_[4].pressed && buttons_[1].on_press) mode_ = Input::Gamepad::LB_B;
            if (buttons_[4].pressed && buttons_[2].on_press) mode_ = Input::Gamepad::LB_X;
            if (buttons_[4].pressed && buttons_[3].on_press) mode_ = Input::Gamepad::LB_Y;
            if (buttons_[4].pressed && buttons_[9].on_press) mode_ = Input::Gamepad::LB_LStick;
            if (buttons_[4].pressed && buttons_[10].on_press) mode_ = Input::Gamepad::LB_RStick;
            if (buttons_[4].pressed && axes_[7] < 0) mode_ = Input::Gamepad::LB_DPadUp;
            if (buttons_[4].pressed && axes_[7] > 0) mode_ = Input::Gamepad::LB_DPadDown;
            if (buttons_[4].pressed && axes_[6] > 0) mode_ = Input::Gamepad::LB_DPadRight;
            if (buttons_[4].pressed && axes_[6] < 0) mode_ = Input::Gamepad::LB_DPadLeft;
            if (buttons_[5].pressed && buttons_[0].on_press) mode_ = Input::Gamepad::RB_A;
            if (buttons_[5].pressed && buttons_[1].on_press) mode_ = Input::Gamepad::RB_B;
            if (buttons_[5].pressed && buttons_[2].on_press) mode_ = Input::Gamepad::RB_X;
            if (buttons_[5].pressed && buttons_[3].on_press) mode_ = Input::Gamepad::RB_Y;
            if (buttons_[5].pressed && buttons_[9].on_press) mode_ = Input::Gamepad::RB_LStick;
            if (buttons_[5].pressed && buttons_[10].on_press) mode_ = Input::Gamepad::RB_RStick;
            if (buttons_[5].pressed && axes_[7] < 0) mode_ = Input::Gamepad::RB_DPadUp;
            if (buttons_[5].pressed && axes_[7] > 0) mode_ = Input::Gamepad::RB_DPadDown;
            if (buttons_[5].pressed && axes_[6] > 0) mode_ = Input::Gamepad::RB_DPadRight;
            if (buttons_[5].pressed && axes_[6] < 0) mode_ = Input::Gamepad::RB_DPadLeft;
            if (buttons_[4].pressed && buttons_[5].on_press) mode_ = Input::Gamepad::LB_RB;
        }
        // Passive stays available after takeover once the device was armed.
        // Unlike the original Go2 sequential map, other keys cannot override it.
        if (was_ready_ && (passive_in_batch_ ||
                          (buttons_[4].pressed && buttons_[2].pressed)))
            mode_ = Input::Gamepad::LB_X;
    }

    void RequireNeutral() noexcept
    {
        ready_ = false;
        mode_ = Input::Gamepad::None;
        passive_in_batch_ = false;
        for (auto& button : buttons_)
        {
            button.on_press = false;
            button.on_release = false;
        }
        // Keep observed device state: joydev sends changes, not heartbeats.
    }

    void Reset() noexcept
    {
        buttons_ = {};
        axes_ = {};
        buttons_seen_ = {};
        axes_seen_ = {};
        ready_ = false;
        was_ready_ = false;
        initial_batch_ = false;
        passive_in_batch_ = false;
        mode_ = Input::Gamepad::None;
    }

    LocalGamepadSample Snapshot(bool connected = true) const noexcept
    {
        LocalGamepadSample sample;
        sample.connected = connected;
        if (!connected)
            return sample;
        sample.ready = ready_;
        sample.passive = !initial_batch_ && was_ready_ &&
                         (passive_in_batch_ ||
                          (buttons_[4].pressed && buttons_[2].pressed));
        sample.mode = sample.passive ? Input::Gamepad::LB_X : mode_;
        if (ready_)
        {
            sample.axes = {-static_cast<float>(axes_[1]) / static_cast<float>(MaxValue),
                           -static_cast<float>(axes_[0]) / static_cast<float>(MaxValue),
                           -static_cast<float>(axes_[3]) / static_cast<float>(MaxValue)};
        }
        return sample;
    }

private:
    bool HaveRequiredStates() const noexcept
    {
        return axes_seen_[0] && axes_seen_[1] && axes_seen_[3] && axes_seen_[7] &&
               buttons_seen_[0] && buttons_seen_[1] && buttons_seen_[2] &&
               buttons_seen_[4] && buttons_seen_[5];
    }

    bool IsNeutral() const noexcept
    {
        return axes_[0] == 0 && axes_[1] == 0 && axes_[3] == 0 && axes_[7] == 0 &&
               !buttons_[0].pressed && !buttons_[1].pressed && !buttons_[2].pressed &&
               !buttons_[4].pressed && !buttons_[5].pressed;
    }

    static constexpr int MaxValue = 1 << (16 - 1);
    static constexpr double Deadzone = 0.05;
    std::array<LocalButton, 20> buttons_{};
    std::array<int, 10> axes_{};
    std::array<bool, 20> buttons_seen_{};
    std::array<bool, 10> axes_seen_{};
    bool ready_ = false;
    bool was_ready_ = false;
    bool initial_batch_ = false;
    bool passive_in_batch_ = false;
    Input::Gamepad mode_ = Input::Gamepad::None;
};

class SimJoystickInput
{
public:
    bool Setup(const std::string& device, int bits = 16)
    {
        device_.reset();
        decoder_.Reset();
        have_poll_time_ = false;
        must_drain_ = false;
        // The existing Linux Joystick and verified simulation use 16-bit axes.
        if (bits != 16)
            return false;
        device_ = std::make_unique<Joystick>(device);
        if (!device_->isFound())
        {
            device_.reset();
            return false;
        }
        return true;
    }

    void Poll(std::chrono::steady_clock::time_point now)
    {
        decoder_.BeginPoll();
        if (!device_ || !device_->isFound())
        {
            Disconnect();
            return;
        }
        if (have_poll_time_ && now >= last_poll_ &&
            now - last_poll_ < std::chrono::milliseconds(10))
        {
            // BeginPoll already consumed the last mode event. Preserve axes
            // and readiness, but do not regenerate held D-pad mode requests.
            return;
        }
        if (have_poll_time_ &&
            (now < last_poll_ || now - last_poll_ > std::chrono::milliseconds(100)))
            decoder_.RequireNeutral();
        have_poll_time_ = true;
        last_poll_ = now;
        bool drained = false;
        for (unsigned attempt = 0; attempt < MaxAttempts; ++attempt)
        {
            JoystickEvent event{};
            // Current sample() calls read() and immediately returns. Resetting
            // errno lets us distinguish its false return without changing it.
            errno = 0;
            if (device_->sample(&event))
            {
                decoder_.Consume(event);
                continue;
            }
            const int error = errno;
            if (error == EINTR)
                continue;
            if (error == EAGAIN || error == EWOULDBLOCK)
            {
                drained = true;
                break;
            }
            // errno==0 also covers EOF/short reads in the current wrapper.
            Disconnect();
            return;
        }
        if (!drained)
        {
            // Do not authorize motion while unread backlog remains. The next
            // drain may finish it; normal empty polls do not imply a disconnect.
            must_drain_ = true;
            decoder_.RequireNeutral();
            return;
        }
        must_drain_ = false;
        decoder_.FinishPoll();
    }

    void RequireNeutral() noexcept
    {
        decoder_.RequireNeutral();
    }

    LocalGamepadSample Snapshot() const noexcept
    {
        return decoder_.Snapshot(device_ && device_->isFound());
    }

private:
    void Disconnect() noexcept
    {
        device_.reset();
        decoder_.Reset();
        have_poll_time_ = false;
        must_drain_ = false;
    }

    static constexpr unsigned MaxAttempts = 256;
    std::unique_ptr<Joystick> device_;
    SimJoystickDecoder decoder_;
    std::chrono::steady_clock::time_point last_poll_{};
    bool have_poll_time_ = false;
    bool must_drain_ = false;
};


struct Velocity
{
    float x = 0.0f, y = 0.0f, yaw = 0.0f;
};

inline bool IsModeEvent(Input::Gamepad event) noexcept
{
    return event == Input::Gamepad::A || event == Input::Gamepad::B ||
           event == Input::Gamepad::RB_DPadUp || event == Input::Gamepad::LB_X;
}

struct ModeDecision
{
    Input::Keyboard keyboard = Input::Keyboard::None;
    Input::Gamepad gamepad = Input::Gamepad::None;
    bool passive = false;
    bool clear_velocity = false;
};

inline ModeDecision DecideMode(Input::Keyboard key, const LocalGamepadSample& local) noexcept
{
    ModeDecision decision;
    decision.passive = key == Input::Keyboard::P || local.passive;
    const bool keyboard_mode = key == Input::Keyboard::Num0 ||
        key == Input::Keyboard::Num1 || key == Input::Keyboard::Num9 ||
        key == Input::Keyboard::P;
    if (decision.passive)
    {
        decision.keyboard = Input::Keyboard::P;
        decision.gamepad = Input::Gamepad::LB_X;
    }
    else
    {
        if (local.connected && local.ready && IsModeEvent(local.mode))
            decision.gamepad = local.mode;
        if (keyboard_mode && decision.gamepad == Input::Gamepad::None)
            decision.keyboard = key;
    }
    // Movement keys never enter the shared controller's +/-0.1 generator.
    decision.clear_velocity = keyboard_mode || IsModeEvent(decision.gamepad) || decision.passive;
    return decision;
}

class VelocityInput
{
public:
    void Configure(const std::array<float, 3>& limits)
    {
        for (float value : limits)
            if (!std::isfinite(value) || value <= 0.0f)
                throw std::runtime_error("BPX invalid velocity input limit");
        limits_ = limits;
        configured_ = true;
        keyboard_ = {};
        was_local_active_ = false;
        was_local_connected_ = false;
    }

    void ClearKeyboard() noexcept { keyboard_ = {}; }

    Velocity Update(Input::Keyboard key, const LocalGamepadSample& local,
                    bool clear_velocity = false)
    {
        if (!configured_) throw std::runtime_error("BPX velocity input not configured");
        bool finite = true;
        for (float value : local.axes) finite = finite && std::isfinite(value);
        const bool usable = local.connected && local.ready && finite;
        const bool active = usable &&
            (local.axes[0] != 0.0f || local.axes[1] != 0.0f || local.axes[2] != 0.0f);
        const bool centered = was_local_active_ && usable && !active;
        const bool lost = was_local_connected_ && !local.connected;
        const bool invalid = local.connected && !finite;
        const bool suppressed = was_local_active_ && !usable;
        const bool clear = clear_velocity || active || centered || lost || invalid || suppressed;
        if (clear) ClearKeyboard();
        if (!clear)
        {
            switch (key)
            {
            case Input::Keyboard::W: keyboard_.x += 0.1f; break;
            case Input::Keyboard::S: keyboard_.x -= 0.1f; break;
            case Input::Keyboard::A: keyboard_.y += 0.1f; break;
            case Input::Keyboard::D: keyboard_.y -= 0.1f; break;
            case Input::Keyboard::Q: keyboard_.yaw += 0.1f; break;
            case Input::Keyboard::E: keyboard_.yaw -= 0.1f; break;
            case Input::Keyboard::Space: ClearKeyboard(); break;
            default: break;
            }
            keyboard_ = Bounded(keyboard_);
        }
        was_local_active_ = active;
        was_local_connected_ = local.connected;
        if (clear_velocity || centered || lost || invalid || suppressed) return {};
        // Preserve MuJoCo raw normalized command, then apply BPX real limits.
        // Do not multiply by the limits: e.g. raw .5 remains .5 with a .7 limit.
        if (active) return Bounded({local.axes[0], local.axes[1], local.axes[2]});
        return keyboard_;
    }

private:
    Velocity Bounded(Velocity value) const noexcept
    {
        value.x = std::clamp(value.x, -limits_[0], limits_[0]);
        value.y = std::clamp(value.y, -limits_[1], limits_[1]);
        value.yaw = std::clamp(value.yaw, -limits_[2], limits_[2]);
        return value;
    }
    std::array<float, 3> limits_{};
    Velocity keyboard_{};
    bool configured_ = false;
    bool was_local_active_ = false;
    bool was_local_connected_ = false;
};

} // namespace bpx_teleop

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
    void SetupSysJoystick();
    void StartLoop();
    void StopLoop() noexcept;
    void ControlLoop() noexcept;
    void BestEffortDamping() noexcept;
    void Close() noexcept;

    bpx_sdk::JointLevelControl sdk_;
    bpx_teleop::SimJoystickInput sys_gamepad_;
    bpx_teleop::VelocityInput velocity_input_;
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