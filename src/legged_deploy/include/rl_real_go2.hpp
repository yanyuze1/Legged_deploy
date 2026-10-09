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

    Velocity Update(Input::Keyboard key, const RemoteSample& remote,
                    TimePoint now, bool clear_keyboard = false) noexcept
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

        // Old keyboard commands never resume after stick takeover, centering,
        // loss of a previously fresh DDS stream, or an invalid axis sample.
        if (clear_keyboard || active || centered || expired || invalid)
            ClearKeyboard();

        if (!clear_keyboard && !active && !centered && !expired && !invalid)
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
        // Preserve the existing remote signs/normal scale, with normalized-axis
        // bounds; keyboard speed limits do not apply to a remote command.
        if (active)
            return {std::clamp(remote.ly, -1.0f, 1.0f),
                    -std::clamp(remote.lx, -1.0f, 1.0f),
                    -std::clamp(remote.rx, -1.0f, 1.0f)};
        return keyboard_;
    }

private:
    Velocity keyboard_{};
    bool was_remote_active_ = false;
    bool was_remote_fresh_ = false;
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
    void StartLoops();
    void StopLoops();
    void RunPeriodic(const char* name, double seconds, std::function<void()> callback);
    void CloseChannels() noexcept;

    // Only the control thread owns control's input fields and input arbitration.
    go2_teleop::SimpleVelocityInput velocity_input_;
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