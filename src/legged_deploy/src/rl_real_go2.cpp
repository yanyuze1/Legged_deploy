/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2.hpp"
#include <stdexcept>
#include <unistd.h>

RL_Real::RL_Real(int argc, char **argv)
{
    try
    {

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
#elif defined(USE_ROS2) && defined(USE_ROS)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
#endif

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "go2";
    this->ReadYaml(this->robot_name, "base.yaml");
    this->SetupSysJoystick();

    // auto load FSM by robot_name
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

    // init robot
    this->InitLowCmd();
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    // create lowcmd publisher
    this->lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    this->lowcmd_publisher->InitChannel();
    // create lowstate subscriber
    this->lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    const auto inbound = this->inbound_;
    this->lowstate_subscriber->InitChannel([inbound](const void* message)
    {
        std::lock_guard<std::mutex> lock(inbound->mutex);
        inbound->low_state = *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    }, 1);
    // create joystick subscriber
    this->joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    this->joystick_subscriber->InitChannel([inbound](const void* message)
    {
        std::lock_guard<std::mutex> lock(inbound->mutex);
        inbound->joystick = *static_cast<const unitree_go::msg::dds_::WirelessController_*>(message);
        inbound->joystick_time = go2_teleop::Clock::now();
        inbound->joystick_received = true;
    }, 1);
    // init MotionSwitcherClient
    this->msc.SetTimeout(10.0f);
    this->msc.Init();
    // Shut down motion control-related service
    while(this->QueryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = this->msc.ReleaseMode();
        if (ret == 0)
        {
            std::cout << "ReleaseMode succeeded." << std::endl;
        }
        else
        {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(1);
    }

    // Initialize optional buffers before the entry-local worker threads start.
#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
    this->StartLoops();
    }
    catch (...)
    {
        // A constructor failure does not call ~RL_Real(). Stop any worker that
        // was already started before member data can be destroyed.
        this->StopLoops();
        this->CloseChannels();
        throw;
    }
}

RL_Real::~RL_Real()
{
    this->StopLoops();
    this->CloseChannels();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::StartLoops()
{
    const double dt = this->params.Get<float>("dt");
    const double model_dt = dt * this->params.Get<int>("decimation");
    const double max_seconds = std::chrono::duration<double>(go2_teleop::Clock::duration::max()).count();
    if (!std::isfinite(dt) || dt <= 0 || dt >= max_seconds ||
        !std::isfinite(model_dt) || model_dt <= 0 || model_dt >= max_seconds)
        throw std::runtime_error("dt and dt*decimation must be finite, positive and representable");
    if (std::chrono::duration_cast<go2_teleop::Clock::duration>(std::chrono::duration<double>(dt)) <=
            go2_teleop::Clock::duration::zero() ||
        std::chrono::duration_cast<go2_teleop::Clock::duration>(std::chrono::duration<double>(model_dt)) <=
            go2_teleop::Clock::duration::zero())
        throw std::runtime_error("loop period is below steady_clock resolution");
    this->control_thread_ = std::thread([this, dt]
    {
        this->RunPeriodic("control", dt, [this] { this->RobotControl(); });
    });
    this->model_thread_ = std::thread([this, model_dt]
    {
        this->RunPeriodic("model", model_dt, [this] { this->RunModel(); });
    });
#ifdef PLOT
    this->plot_thread_ = std::thread([this]
    {
        this->RunPeriodic("plot", 0.002, [this] { this->Plot(); });
    });
#endif
}

void RL_Real::StopLoops()
{
    this->stop_requested_.store(true);
    this->stop_cv_.notify_all();
    // StopLoops is called only by the constructor's catch or its owning thread.
    // Never hold the input/velocity mutex while waiting for worker completion.
    if (this->control_thread_.joinable()) this->control_thread_.join();
    if (this->model_thread_.joinable()) this->model_thread_.join();
#ifdef PLOT
    if (this->plot_thread_.joinable()) this->plot_thread_.join();
#endif
}

void RL_Real::RunPeriodic(const char* name, double seconds, std::function<void()> callback)
{
    const auto period = std::chrono::duration_cast<go2_teleop::Clock::duration>(
        std::chrono::duration<double>(seconds));
    try
    {
        while (!this->stop_requested_.load())
        {
            const auto started = go2_teleop::Clock::now();
            callback();
            std::unique_lock<std::mutex> lock(this->stop_mutex_);
            this->stop_cv_.wait_until(lock, started + period,
                [this] { return this->stop_requested_.load(); });
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << LOGGER::ERROR << "Real " << name << " loop stopped: " << error.what() << std::endl;
        this->stop_requested_.store(true);
        this->stop_cv_.notify_all();
    }
    catch (...)
    {
        std::cerr << LOGGER::ERROR << "Real " << name << " loop stopped: unknown exception" << std::endl;
        this->stop_requested_.store(true);
        this->stop_cv_.notify_all();
    }
    // This is lifecycle cleanup, not a watchdog or guaranteed robot stop.
}

void RL_Real::CloseChannels() noexcept
{
    // CloseChannel can wait for a callback; hold neither input nor velocity lock.
    const auto close = [](const auto& channel)
    {
        try
        {
            if (channel) channel->CloseChannel();
        }
        catch (const std::exception& error)
        {
            std::cerr << LOGGER::ERROR << "Closing DDS channel failed: " << error.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << LOGGER::ERROR << "Closing DDS channel failed" << std::endl;
        }
    };
    close(this->lowstate_subscriber);
    close(this->joystick_subscriber);
    close(this->lowcmd_publisher);
}

void RL_Real::ResetInputEvents()
{
    // ClearInput() restores last_keyboard. Explicitly consume all four fields.
    this->control.current_keyboard = Input::Keyboard::None;
    this->control.last_keyboard = Input::Keyboard::None;
    this->control.current_gamepad = Input::Gamepad::None;
    this->control.last_gamepad = Input::Gamepad::None;
}

void RL_Real::SetupSysJoystick()
{
    // Use the same device, precision and mapping as the validated MuJoCo path.
    const std::string device = "/dev/input/js0";
    if (this->sys_gamepad_.Setup(device, 16))
        std::cout << LOGGER::INFO << "[Joystick] Opened " << device
                  << "; center sticks and release buttons before use" << std::endl;
    else
        std::cout << LOGGER::NOTE << "Joystick [" << device
                  << "] open failed; official remote and keyboard remain available"
                  << std::endl;
}

void RL_Real::MapGamepad()
{
    if (this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::A);
    if (this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::B);
    if (this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::X);
    if (this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->unitree_joy.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

}

void RL_Real::GetState(RobotState<float> *state)
{
    {
        std::lock_guard<std::mutex> lock(this->inbound_->mutex);
        this->unitree_low_state = this->inbound_->low_state;
        this->joystick = this->inbound_->joystick;
        this->joystick_time_ = this->inbound_->joystick_time;
        this->joystick_received_ = this->inbound_->joystick_received;
    }
    this->unitree_joy.value = this->joystick.keys();
    // GetState assembles telemetry only. Movement/buttons are arbitrated below.
    state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0]; // w
    state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1]; // x
    state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2]; // y
    state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3]; // z

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];
    }
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        state->motor_state.q[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q();
        state->motor_state.dq[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq();
        state->motor_state.tau_est[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est();
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    unitree_go::msg::dds_::LowCmd_ dds_low_command;
    dds_low_command.head()[0] = 0xFE;
    dds_low_command.head()[1] = 0xEF;
    dds_low_command.level_flag() = 0xFF;
    dds_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        dds_low_command.motor_cmd()[i].mode() = 0x01;
        dds_low_command.motor_cmd()[i].q() = PosStopF;
        dds_low_command.motor_cmd()[i].kp() = 0;
        dds_low_command.motor_cmd()[i].dq() = VelStopF;
        dds_low_command.motor_cmd()[i].kd() = 0;
        dds_low_command.motor_cmd()[i].tau() = 0;
    }

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].mode() = 0x01;
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q() = command->motor_command.q[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq() = command->motor_command.dq[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp() = command->motor_command.kp[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd() = command->motor_command.kd[i];
        dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau() = command->motor_command.tau[i];
    }

    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(dds_low_command);

#ifdef PLOT
    this->unitree_low_command = dds_low_command;
#endif
}

void RL_Real::RobotControl()
{
    this->ResetInputEvents();
    if (::isatty(STDIN_FILENO) && ::tcgetpgrp(STDIN_FILENO) == ::getpgrp())
        this->KeyboardInterface();
    const auto keyboard = this->control.current_keyboard;
    this->ResetInputEvents();
    this->GetState(&this->robot_state);

    const auto now = go2_teleop::Clock::now();
    const bool local_was_connected = this->sys_gamepad_.Snapshot().connected;
    this->sys_gamepad_.Poll(now);
    auto local = this->sys_gamepad_.Snapshot();
    if (local_was_connected && !local.connected)
        std::cout << std::endl << LOGGER::NOTE
                  << "[PC gamepad] Disconnected; local velocity cleared. Reconnect and restart to reopen."
                  << std::endl;
    const go2_teleop::RemoteSample remote{this->joystick.ly(), this->joystick.lx(),
        this->joystick.rx(), this->joystick_received_, this->joystick_time_};
    const bool remote_fresh = go2_teleop::SimpleVelocityInput::IsFresh(remote, now);
    const bool remote_finite = std::isfinite(remote.ly) && std::isfinite(remote.lx) &&
                               std::isfinite(remote.rx);
    const bool remote_active = remote_fresh && remote_finite &&
        (std::fabs(remote.ly) >= go2_teleop::SimpleVelocityInput::Deadzone ||
         std::fabs(remote.lx) >= go2_teleop::SimpleVelocityInput::Deadzone ||
         std::fabs(remote.rx) >= go2_teleop::SimpleVelocityInput::Deadzone);
    const bool remote_lost = this->official_was_fresh_ && !remote_fresh;
    this->official_was_fresh_ = remote_fresh;
    const bool official_has_control = remote_fresh &&
        (remote_active || this->joystick.keys() != 0);
    if (remote_fresh) this->MapGamepad();
    const auto official_event = this->control.current_gamepad;
    // Stop requests from any source are handled before ordinary mode arbitration.
    const bool official_passive = remote_fresh && this->unitree_joy.components.L1 &&
                                  this->unitree_joy.components.X;
    // Preserve local Passive even when ordinary local input is suppressed.
    const bool local_passive = local.passive;
    if (official_has_control || remote_lost || (remote_fresh && !remote_finite))
    {
        this->sys_gamepad_.RequireNeutral();
        local = this->sys_gamepad_.Snapshot();
    }
    local.passive = local_passive;
    const auto decision = go2_teleop::DecideMode(
        keyboard, official_event, official_has_control, official_passive, local);
    bool passive_requested = decision.passive;
    this->control.current_keyboard = decision.keyboard;
    this->control.current_gamepad = decision.gamepad;
    const auto request_passive = [this]()
    {
        this->control.current_keyboard = Input::Keyboard::P;
        this->control.current_gamepad = Input::Gamepad::LB_X;
        this->velocity_input_.ClearKeyboard();
        if (this->fsm.current_state_ &&
            this->fsm.current_state_->GetStateName() == "RLFSMStatePassive")
        {
            this->fsm.next_state_ = this->fsm.current_state_;
            this->fsm.mode_ = FSM::Mode::NORMAL;
        }
        else
            this->fsm.RequestStateChange("RLFSMStatePassive");
    };
    if (passive_requested) request_passive();
    bool clear_input = decision.clear_velocity;
    if (clear_input)
    {
        this->control.x = this->control.y = this->control.yaw = 0.0f;
        std::lock_guard<std::mutex> lock(this->velocity_mutex_);
        this->published_velocity_ = {};
    }
    const auto previous_state = this->fsm.current_state_;
    this->StateController(&this->robot_state, &this->robot_command);
    const bool changed_state = previous_state != this->fsm.current_state_;
    // Normal frames poll once so mode edges are consumed by the next heartbeat.
    // A long model load needs a fresh state snapshot; ordinary events from that
    // gap are discarded by Reader's RequireNeutral, while Passive is retained.
    const auto after_fsm = go2_teleop::Clock::now();
    if (after_fsm - now > std::chrono::milliseconds(100))
    {
        const bool was_connected = this->sys_gamepad_.Snapshot().connected;
        this->sys_gamepad_.Poll(after_fsm);
        local = this->sys_gamepad_.Snapshot();
        if (was_connected && !local.connected)
            std::cout << std::endl << LOGGER::NOTE
                      << "[PC gamepad] Disconnected during model load; local velocity cleared."
                      << std::endl;
        if (local.passive && !passive_requested)
        {
            passive_requested = true;
            clear_input = true;
            request_passive();
            this->rl_init_done = false;
        }
    }
    if (clear_input || changed_state || official_has_control || remote_lost ||
        (remote_fresh && !remote_finite) || keyboard == Input::Keyboard::Space)
        this->sys_gamepad_.RequireNeutral();
    local = this->sys_gamepad_.Snapshot();
    auto velocity = this->velocity_input_.Update(keyboard, remote, local, go2_teleop::Clock::now(),
        clear_input || changed_state, official_has_control);
    if (passive_requested)
    {
        velocity = {};
        // Software damping, including cancellation of a pending GetUp.
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            this->robot_command.motor_command.dq[i] = 0;
            this->robot_command.motor_command.kp[i] = 0;
            this->robot_command.motor_command.kd[i] = 8;
            this->robot_command.motor_command.tau[i] = 0;
        }
    }
    this->control.x = velocity.x;
    this->control.y = velocity.y;
    this->control.yaw = velocity.yaw;
    {
        std::lock_guard<std::mutex> lock(this->velocity_mutex_);
        this->published_velocity_ = velocity;
    }
    this->ResetInputEvents();
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        go2_teleop::Velocity velocity;
        {
            std::lock_guard<std::mutex> lock(this->velocity_mutex_);
            velocity = this->published_velocity_;
        }
        this->obs.commands = {velocity.x, velocity.y, velocity.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};

        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    unitree_go::msg::dds_::LowState_ low_state;
    {
        std::lock_guard<std::mutex> lock(this->inbound_->mutex);
        low_state = this->inbound_->low_state;
    }
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(low_state.motor_state()[i].q());
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

uint32_t RL_Real::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RL_Real::InitLowCmd()
{
    this->unitree_low_command.head()[0] = 0xFE;
    this->unitree_low_command.head()[1] = 0xEF;
    this->unitree_low_command.level_flag() = 0xFF;
    this->unitree_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        this->unitree_low_command.motor_cmd()[i].q() = (PosStopF);
        this->unitree_low_command.motor_cmd()[i].kp() = (0);
        this->unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
        this->unitree_low_command.motor_cmd()[i].kd() = (0);
        this->unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

int RL_Real::QueryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = this->msc.CheckMode(robotForm, motionName);
    if (ret == 0)
    {
        std::cout << "CheckMode succeeded." << std::endl;
    }
    else
    {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if (motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = QueryServiceName(robotForm, motionName);
        std::cout << "Service: " << serviceName << " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string RL_Real::QueryServiceName(std::string form, std::string name)
{
    if (form == "0")
    {
        if (name == "normal" )   return "sport_mode";
        if (name == "ai" )       return "ai_sport";
        if (name == "advanced" ) return "advanced_sport";
    }
    else
    {
        if (name == "ai-w" )     return "wheeled_sport(go2W)";
        if (name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

#if !defined(USE_CMAKE) && defined(USE_ROS)
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1) && defined(USE_ROS)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2) && defined(USE_ROS)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1) && defined(USE_ROS)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#elif defined(USE_CMAKE) || !defined(USE_ROS)
// Signal handler for CMAKE mode
volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int)
{
    // A signal handler may only set this flag; logging occurs in normal code.
    g_shutdown_requested = 1;
}
#endif

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " networkInterface" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    ChannelFactory::Instance()->Init(0, argv[1]);

#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#elif defined(USE_CMAKE) || !defined(USE_ROS)
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    RL_Real rl_sar(argc, argv);
    while (!g_shutdown_requested && rl_sar.IsRunning()) { sleep(1); }
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;
    if (!rl_sar.IsRunning()) return 1;
#endif

    return 0;
}