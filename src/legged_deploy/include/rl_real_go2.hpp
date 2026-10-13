/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_GO2_HPP
#define RL_REAL_GO2_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "fsm_go2.hpp"
#include "joystick.hh"
#include <array>
#include <cerrno>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// Pure input arbitration. The initial keyboard limits require confirmation
// against the policy's training command range before real-robot acceptance.
namespace go2_teleop
{
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

struct Velocity
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

struct RemoteSample
{
    float ly = 0.0f;
    float lx = 0.0f;
    float rx = 0.0f;
    bool received = false;
    TimePoint received_at{};
};

// Insert inside go2_teleop. This reuses the project's existing Joystick class.
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


inline bool IsModeEvent(Input::Gamepad event) noexcept
{
    return event == Input::Gamepad::A || event == Input::Gamepad::B ||
           event == Input::Gamepad::RB_DPadUp || event == Input::Gamepad::LB_X ||
           event == Input::Gamepad::X;
}

struct ModeDecision
{
    Input::Keyboard keyboard = Input::Keyboard::None;
    Input::Gamepad gamepad = Input::Gamepad::None;
    bool passive = false;
    bool clear_velocity = false;
};

inline ModeDecision DecideMode(Input::Keyboard keyboard, Input::Gamepad official,
                               bool official_has_control, bool official_passive,
                               const LocalGamepadSample& local) noexcept
{
    ModeDecision result;
    result.passive = keyboard == Input::Keyboard::P || official_passive || local.passive;
    const bool movement = keyboard == Input::Keyboard::W || keyboard == Input::Keyboard::S ||
        keyboard == Input::Keyboard::A || keyboard == Input::Keyboard::D ||
        keyboard == Input::Keyboard::Q || keyboard == Input::Keyboard::E ||
        keyboard == Input::Keyboard::Space;
    const bool keyboard_mode = keyboard == Input::Keyboard::Num0 ||
        keyboard == Input::Keyboard::Num1 || keyboard == Input::Keyboard::Num9 ||
        keyboard == Input::Keyboard::P || keyboard == Input::Keyboard::N;
    if (result.passive)
    {
        result.keyboard = Input::Keyboard::P;
        result.gamepad = Input::Gamepad::LB_X;
    }
    else
    {
        if (official_has_control)
        {
            if (IsModeEvent(official)) result.gamepad = official;
        }
        else if (local.ready)
        {
            result.gamepad = local.mode;
        }
        if (!movement && !(keyboard_mode && (official_has_control || IsModeEvent(result.gamepad))))
            result.keyboard = keyboard;
    }
    result.clear_velocity = keyboard_mode || IsModeEvent(result.gamepad) || result.passive;
    return result;
}

class SimpleVelocityInput
{
public:
    static constexpr float Step = 0.1f;
    static constexpr float XLimit = 0.3f;
    static constexpr float YLimit = 0.2f;
    static constexpr float YawLimit = 0.5f;
    static constexpr float Deadzone = 0.05f;

    static bool IsFresh(const RemoteSample& remote, TimePoint now) noexcept
    {
        return remote.received && now >= remote.received_at &&
               now - remote.received_at < std::chrono::milliseconds(500);
    }

    void ClearKeyboard() noexcept
    {
        keyboard_ = {};
    }

    // Preserve the original two-source interface for existing callers/tests.
    Velocity Update(Input::Keyboard key, const RemoteSample& remote,
                    TimePoint now, bool clear_keyboard = false) noexcept
    {
        return Update(key, remote, LocalGamepadSample{}, now, clear_keyboard, false);
    }

    Velocity Update(Input::Keyboard key, const RemoteSample& remote,
                    const LocalGamepadSample& local, TimePoint now,
                    bool clear_keyboard = false,
                    bool official_has_control = false) noexcept
    {
        const bool fresh = IsFresh(remote, now);
        const bool finite = std::isfinite(remote.ly) &&
                            std::isfinite(remote.lx) && std::isfinite(remote.rx);
        const bool active = fresh && finite &&
            (std::fabs(remote.ly) >= Deadzone || std::fabs(remote.lx) >= Deadzone ||
             std::fabs(remote.rx) >= Deadzone);
        const bool centered = was_remote_active_ && fresh && finite && !active;
        const bool expired = was_remote_fresh_ && !fresh;
        const bool invalid = fresh && !finite;
        bool local_finite = true;
        for (size_t i = 0; i < 3; ++i)
            local_finite = local_finite && std::isfinite(local.axes[i]) &&
                           std::isfinite(local.limits[i]) && local.limits[i] >= 0.0f;
        const bool local_usable = local.connected && local.ready && local_finite;
        const bool local_active = local_usable &&
            (local.axes[0] != 0.0f || local.axes[1] != 0.0f || local.axes[2] != 0.0f);
        const bool local_centered = was_local_active_ && local_usable && !local_active;
        const bool local_lost = was_local_connected_ && !local.connected;
        const bool local_invalid = local.connected && !local_finite;
        const bool local_suppressed = was_local_active_ && !local_usable;

        const bool clear = clear_keyboard || official_has_control || active ||
            centered || expired || invalid || local_active || local_centered ||
            local_lost || local_invalid || local_suppressed;
        if (clear) ClearKeyboard();
        if (!clear)
        {
            switch (key)
            {
            case Input::Keyboard::W: keyboard_.x += Step; break;
            case Input::Keyboard::S: keyboard_.x -= Step; break;
            case Input::Keyboard::A: keyboard_.y += Step; break;
            case Input::Keyboard::D: keyboard_.y -= Step; break;
            case Input::Keyboard::Q: keyboard_.yaw += Step; break;
            case Input::Keyboard::E: keyboard_.yaw -= Step; break;
            case Input::Keyboard::Space: ClearKeyboard(); break;
            default: break;
            }
            keyboard_.x = std::clamp(keyboard_.x, -XLimit, XLimit);
            keyboard_.y = std::clamp(keyboard_.y, -YLimit, YLimit);
            keyboard_.yaw = std::clamp(keyboard_.yaw, -YawLimit, YawLimit);
        }
        was_remote_active_ = active;
        was_remote_fresh_ = fresh;
        was_local_active_ = local_active;
        was_local_connected_ = local.connected;

        // Official stick signs and limits remain the same as the current entry.
        if (active)
            return {std::clamp(remote.ly, -1.0f, 1.0f),
                    -std::clamp(remote.lx, -1.0f, 1.0f),
                    -std::clamp(remote.rx, -1.0f, 1.0f)};
        if (official_has_control || clear_keyboard || centered || expired || invalid ||
            local_centered || local_lost || local_invalid || local_suppressed)
            return {};
        if (local_active)
            return {std::clamp(local.axes[0], -1.0f, 1.0f) * local.limits[0],
                    std::clamp(local.axes[1], -1.0f, 1.0f) * local.limits[1],
                    std::clamp(local.axes[2], -1.0f, 1.0f) * local.limits[2]};
        return keyboard_;
    }

private:
    Velocity keyboard_{};
    bool was_remote_active_ = false;
    bool was_remote_fresh_ = false;
    bool was_local_active_ = false;
    bool was_local_connected_ = false;
};
} // namespace go2_teleop

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();
    bool IsRunning() const noexcept { return !stop_requested_.load(); }

#if defined(USE_ROS2) && defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    void ResetInputEvents();
    void MapGamepad();
    void SetupSysJoystick();
    void StartLoops();
    void StopLoops();
    void RunPeriodic(const char* name, double seconds, std::function<void()> callback);
    void CloseChannels() noexcept;

    // Only the control thread owns control's input fields and input arbitration.
    go2_teleop::SimpleVelocityInput velocity_input_;
    go2_teleop::SimJoystickInput sys_gamepad_;
    bool official_was_fresh_ = false;
    std::mutex velocity_mutex_;
    go2_teleop::Velocity published_velocity_{};

    struct Inbound
    {
        std::mutex mutex;
        unitree_go::msg::dds_::LowState_ low_state{};
        unitree_go::msg::dds_::WirelessController_ joystick{};
        go2_teleop::TimePoint joystick_time{};
        bool joystick_received = false;
    };
    // DDS lambdas retain only Inbound, not this, including during construction.
    std::shared_ptr<Inbound> inbound_ = std::make_shared<Inbound>();
    // The following cached input fields are written/read only by RobotControl.
    unitree_go::msg::dds_::LowState_ unitree_low_state{};
    unitree_go::msg::dds_::WirelessController_ joystick{};
    go2_teleop::TimePoint joystick_time_{};
    bool joystick_received_ = false;
    xKeySwitchUnion unitree_joy{};

    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    void InitLowCmd();
    int QueryMotionStatus();
    std::string QueryServiceName(std::string form, std::string name);
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);
    MotionSwitcherClient msc;
    unitree_go::msg::dds_::LowCmd_ unitree_low_command{};
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;

    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif

    // Entry-local joinable loops; shared LoopFunc and MuJoCo remain unchanged.
    std::atomic<bool> stop_requested_{false};
    std::mutex stop_mutex_;
    std::condition_variable stop_cv_;
    std::thread control_thread_;
    std::thread model_thread_;
#ifdef PLOT
    std::thread plot_thread_;
#endif
};

#endif // RL_REAL_GO2_HPP