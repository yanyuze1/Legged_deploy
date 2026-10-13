/*
 * Copyright (c) 2026 Ercong Huang
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rl_real_bpx.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace
{
volatile std::sig_atomic_t shutdown_requested = 0;
void SignalHandler(int) { shutdown_requested = 1; }

using Quat = std::array<float, 4>; // wxyz
using Vec3 = std::array<float, 3>;

template <size_t N>
void RequireFinite(const std::array<float, N>& values, const char* name)
{
    for (float value : values)
        if (!std::isfinite(value)) throw std::runtime_error(std::string("Nonfinite ") + name);
}

Quat Normalize(Quat value)
{
    RequireFinite(value, "quaternion");
    float norm = 0.0f;
    for (float v : value) norm += v * v;
    norm = std::sqrt(norm);
    if (!std::isfinite(norm) || norm < 0.5f || norm > 1.5f)
        throw std::runtime_error("Quaternion norm outside expected range");
    for (float& v : value) v /= norm;
    return value;
}

Quat Conjugate(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }
Quat Multiply(const Quat& a, const Quat& b)
{
    return {
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    };
}
Vec3 Rotate(const Quat& q, const Vec3& v)
{
    const auto p = Multiply(Multiply(q, {0.0f, v[0], v[1], v[2]}), Conjugate(q));
    return {p[1], p[2], p[3]};
}

template <size_t N>
std::array<float, N> ReadArray(const YAML::Node& config, const char* key)
{
    const auto values = config[key].as<std::vector<float>>();
    if (values.size() != N) throw std::runtime_error(std::string("Wrong array length: ") + key);
    std::array<float, N> result{};
    std::copy(values.begin(), values.end(), result.begin());
    RequireFinite(result, key);
    return result;
}

float Positive(const YAML::Node& config, const char* key)
{
    const float value = config[key].as<float>();
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error(std::string("Invalid positive parameter: ") + key);
    return value;
}

// Cached identity/control-source feedback. Unknown is a received enum value;
// unavailable means the getter did not provide a value.
std::string SdkStatus(const bpx_sdk::RequestRobotState& sdk)
{
    std::ostringstream out;
    bpx_sdk::RobotModel model = bpx_sdk::RobotModel::Unknown;
    bpx_sdk::ControlMode mode = bpx_sdk::ControlMode::Unknown;
    std::string sn;
    uint16_t major = 0, minor = 0, patch = 0;
    out << "connected=" << sdk.isConnected() << " model=";
    if (sdk.getRobotModel(&model))
    {
        const char* name = "Unknown";
        switch (model)
        {
            case bpx_sdk::RobotModel::BPX: name = "BPX"; break;
            case bpx_sdk::RobotModel::BPXPro: name = "BPX-Pro"; break;
            case bpx_sdk::RobotModel::BPW: name = "BPW"; break;
            default: break;
        }
        out << name << '(' << static_cast<unsigned>(model) << ')';
    }
    else out << "<unavailable>";
    out << " SN=";
    if (sdk.getRobotSerialNumber(&sn)) out << (sn.empty() ? "<empty>" : "<available>");
    else out << "<unavailable>";
    out << " control_mode=";
    if (sdk.getControlMode(&mode))
    {
        const char* name = "Unknown";
        if (mode == bpx_sdk::ControlMode::RemoteControl) name = "RemoteControl";
        if (mode == bpx_sdk::ControlMode::Navigator) name = "Navigator";
        out << name << '(' << static_cast<unsigned>(mode) << ')';
    }
    else out << "<unavailable>";
    out << " firmware=";
    if (sdk.getRobotVersion(&major, &minor, &patch)) out << major << '.' << minor << '.' << patch;
    else out << "<unavailable>";
    return out.str();
}

// Public getters do not expose an atomic packet. Reading the sequence on both
// sides detects an update during these calls, but is not a firmware guarantee.
template <typename SnapshotType>
bool ReadHighRateSnapshot(bpx_sdk::JointLevelControl& sdk, SnapshotType& out)
{
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        SnapshotType candidate;
        uint32_t before = 0, after = 0;
        if (sdk.getJointStateSeqHighRate(&before) &&
            sdk.getJointPositionHighRate(candidate.q.data()) &&
            sdk.getJointVelocityHighRate(candidate.dq.data()) &&
            sdk.getJointTorqueHighRate(candidate.tau.data()) &&
            sdk.getImuQuatHighRate(candidate.quat.data()) &&
            sdk.getImuOmegaHighRate(candidate.omega.data()) &&
            sdk.getImuAccHighRate(candidate.acc.data()) &&
            sdk.getJointStateSeqHighRate(&after) && before == after)
        {
            candidate.seq = after;
            out = candidate;
            return true;
        }
    }
    return false;
}

struct ProbeSnapshot
{
    std::array<float, 12> q{}, dq{}, tau{};
    std::array<float, 4> quat{};
    std::array<float, 3> omega{}, acc{};
    uint32_t seq = 0;
};

// This opens a JointControl subscription/session. It submits no motor command,
// but connect() is not a promise that firmware ownership remains unchanged.
// Stop other robot sessions before running this diagnostic.
int JointMonitor(const char* ip)
{
    using DiagnosticClock = std::chrono::steady_clock;
    bpx_sdk::JointLevelControl sdk;
    sdk.setRobotIp(ip);
    sdk.setRobotStateUploadPort(bpx_sdk::DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
    sdk.setJointStateUploadPort(bpx_sdk::DEFAULT_CLIENT_JOINT_STATE_UDP_PORT);
    sdk.setTcpLocalPort(0);
    sdk.setRobotStateUploadRate(100);
    try
    {
        std::cout << LOGGER::NOTE
                  << "BPX JointControl diagnostic: no motor commands; "
                     "session/mode handshake and HighRate reception only. "
                     "It does not restore official control on exit." << std::endl;
        if (!sdk.connect()) throw std::runtime_error("JointControl subscription could not start");
        const auto started = DiagnosticClock::now();
        auto next_print = started;
        auto last_new = started;
        ProbeSnapshot snapshot;
        bool connected_once = false, have_snapshot = false, advancing = false;
        uint64_t observed_updates = 0;
        while (!shutdown_requested)
        {
            const auto now = DiagnosticClock::now();
            const bool connected = sdk.isConnected();
            if (connected_once && !connected)
                throw std::runtime_error("JointControl connection lost; end diagnostic and retry explicitly");
            connected_once = connected_once || connected;
            ProbeSnapshot candidate;
            if (connected && ReadHighRateSnapshot(sdk, candidate))
            {
                RequireFinite(candidate.q, "HighRate joint position");
                RequireFinite(candidate.dq, "HighRate joint velocity");
                RequireFinite(candidate.tau, "HighRate joint torque");
                RequireFinite(candidate.omega, "HighRate IMU angular velocity");
                RequireFinite(candidate.acc, "HighRate IMU acceleration");
                Normalize({candidate.quat[3], candidate.quat[0], candidate.quat[1], candidate.quat[2]});
                if (!have_snapshot || candidate.seq != snapshot.seq)
                {
                    if (have_snapshot && static_cast<int32_t>(candidate.seq - snapshot.seq) <= 0)
                        throw std::runtime_error("HighRate sequence moved backwards; retry explicitly");
                    advancing = have_snapshot;
                    snapshot = candidate;
                    have_snapshot = true;
                    last_new = DiagnosticClock::now();
                    ++observed_updates;
                }
            }
            if (now >= next_print)
            {
                std::cout << "[BPX joint-monitor] " << SdkStatus(sdk)
                          << " high_rate=" << have_snapshot;
                if (have_snapshot)
                {
                    std::cout << " seq=" << snapshot.seq << " advancing=" << advancing
                              << " observed_updates=" << observed_updates << " SDK order q=";
                    for (float value : snapshot.q) std::cout << value << ' ';
                    std::cout << "dq=";
                    for (float value : snapshot.dq) std::cout << value << ' ';
                    std::cout << "quat(xyzw)=";
                    for (float value : snapshot.quat) std::cout << value << ' ';
                    std::cout << "omega=";
                    for (float value : snapshot.omega) std::cout << value << ' ';
                }
                std::cout << std::endl;
                next_print = now + std::chrono::seconds(1);
            }
            if (!advancing && now - started > std::chrono::seconds(10))
                throw std::runtime_error("TCP/advancing HighRate not ready within 10 s; verify JointControl firmware support and DevelopingState");
            if (have_snapshot && DiagnosticClock::now() - last_new > std::chrono::milliseconds(100))
                throw std::runtime_error("HighRate sequence did not advance for 100 ms; no motor commands submitted");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        sdk.disconnect();
        return 0;
    }
    catch (...)
    {
        try { sdk.disconnect(); } catch (...) {}
        throw;
    }
}

// StateQuery uses ordinary telemetry only. JointLevelControl::connect is not
// used by this mode, and no joint/motion command is submitted here.
int Monitor(const char* ip)
{
    bpx_sdk::RequestRobotState state;
    state.setRobotIp(ip);
    state.setRobotStateUploadPort(bpx_sdk::DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
    state.setTcpLocalPort(0);
    state.setRobotStateUploadRate(100);
    if (!state.connect()) throw std::runtime_error("State subscription could not start");
    const auto started = std::chrono::steady_clock::now();
    bool connected_once = false;
    auto next_status = started;
    while (!shutdown_requested)
    {
        const bool connected = state.isConnected();
        connected_once = connected_once || connected;
        if (!connected_once && std::chrono::steady_clock::now() - started > std::chrono::seconds(10))
        {
            state.disconnect();
            throw std::runtime_error("State-query connection timed out");
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_status)
        {
            std::cout << "[BPX monitor] " << SdkStatus(state) << std::endl;
            next_status = now + std::chrono::seconds(1);
        }
        std::array<float, 12> q{}, dq{};
        std::array<float, 4> quat{};
        std::array<float, 3> omega{};
        uint32_t joint_time = 0, imu_time = 0;
        if (connected && state.getJointPosition(q.data()) && state.getJointVelocity(dq.data()) &&
            state.getImuQuat(quat.data()) && state.getImuOmega(omega.data()))
        {
            state.getJointStateTimestamp(&joint_time);
            state.getImuTimestamp(&imu_time);
            std::cout << "[BPX monitor] SDK order q=";
            for (float v : q) std::cout << v << ' ';
            std::cout << "dq=";
            for (float v : dq) std::cout << v << ' ';
            std::cout << "quat(xyzw)=";
            for (float v : quat) std::cout << v << ' ';
            std::cout << "omega=";
            for (float v : omega) std::cout << v << ' ';
            std::cout << "joint_ms=" << joint_time << " imu_ms=" << imu_time << std::endl;
        }
        else
        {
            std::cout << "[BPX monitor] waiting for ordinary telemetry; connected="
                      << connected << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    state.disconnect();
    return 0;
}
}

RL_Real_Bpx::RL_Real_Bpx(const char* robot_ip)
{
    try
    {
        LoadConfiguration();
        PreloadPolicy(); // disk loading and first inference precede control ownership
        if (shutdown_requested) throw std::runtime_error("Startup interrupted");
        SetupTerminal();
        SetupSysJoystick();
        ConnectAndWait(robot_ip);
        for (auto& pair : fsm.states_)
        {
            const auto state = std::dynamic_pointer_cast<RLFSMState>(pair.second);
            if (!state) throw std::runtime_error("Unexpected BPX FSM state type");
            state->fsm_state = &robot_state;
            state->fsm_command = &robot_command;
        }
        bpx_fsm::Damping(*this, &robot_state, &robot_command);
        SetCommand(&robot_command);
        std::cout << LOGGER::INFO
                  << "BPX real ready: 0 stand, 1 policy, 9 return to startup pose, P passive; "
                     "W/S A/D Q/E velocity, Space zero; gamepad A stand, RB+Up policy, "
                     "B down, LB+X passive, sticks velocity; Ctrl+C exit." << std::endl;
        StartLoop();
    }
    catch (...)
    {
        Close();
        throw;
    }
}

RL_Real_Bpx::~RL_Real_Bpx() { Close(); }

void RL_Real_Bpx::LoadConfiguration()
{
    robot_name = "bpx";
    config_name = "mirrorme";
    ang_vel_axis = "body";
    const auto base_file = YAML::LoadFile(
        std::string(POLICY_DIR) + "/bpx/base.yaml");
    if (!base_file[robot_name] || !base_file[robot_name].IsMap())
        throw std::runtime_error("Missing YAML mapping bpx in policy/bpx/base.yaml");
    ReadYaml(robot_name, "base.yaml");
    // The SDK adapter keeps the robot-level settings; policy values merge next.
    base_config_ = YAML::Clone(params.config_node);
    const std::string profile = robot_name + "/" + config_name;
    const auto policy_file = YAML::LoadFile(
        std::string(POLICY_DIR) + "/" + profile + "/config.yaml");
    if (!policy_file[profile] || !policy_file[profile].IsMap())
        throw std::runtime_error("Missing YAML mapping bpx/mirrorme");
    ReadYaml(profile, "config.yaml");

    if (params.Get<int>("num_of_dofs") != 12 ||
        std::fabs(params.Get<float>("dt") - 0.005f) > 1e-7f ||
        params.Get<int>("decimation") != 4 || params.Get<int>("num_observations") != 45 ||
        !params.Get<std::vector<int>>("observations_history").empty() ||
        !params.Get<std::vector<int>>("wheel_indices").empty())
        throw std::runtime_error("BPX expects 12 joints, 45 observations, dt=.005, decimation=4, no history/wheels");
    if (params.Get<std::vector<std::string>>("observations") !=
        std::vector<std::string>{"ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions"})
        throw std::runtime_error("BPX observation order mismatch");
    const auto mapping = params.Get<std::vector<int>>("joint_mapping");
    const std::array<int, 12> expected{0,3,6,9,1,4,7,10,2,5,8,11};
    if (mapping.size() != 12 || !std::equal(mapping.begin(), mapping.end(), expected.begin()))
        throw std::runtime_error("BPX policy-to-SDK joint mapping mismatch");
    std::copy(mapping.begin(), mapping.end(), mapping_.begin());
    sign_ = ReadArray<12>(base_config_, "joint_sign");
    offset_ = ReadArray<12>(base_config_, "joint_offset");
    for (float sign : sign_)
        if (std::fabs(std::fabs(sign) - 1.0f) > 1e-6f)
            throw std::runtime_error("joint_sign values must be +1 or -1");
    velocity_limits_ = ReadArray<3>(base_config_, "real_command_limits");
    for (float limit : velocity_limits_)
        if (limit <= 0.0f) throw std::runtime_error("Invalid operator command limit");
    velocity_input_.Configure(velocity_limits_);
    body_from_imu_ = Normalize(ReadArray<4>(base_config_, "body_from_imu_quat_wxyz"));
    gyro_scale_ = Positive(base_config_, "gyro_scale");
    const std::string omega_frame = base_config_["imu_omega_frame"].as<std::string>();
    if (omega_frame != "imu" && omega_frame != "world")
        throw std::runtime_error("imu_omega_frame must be imu or world");
    omega_in_world_ = (omega_frame == "world");
    state_timeout_ms_ = base_config_["state_timeout_ms"].as<int>();
    connect_timeout_ms_ = base_config_["connect_timeout_ms"].as<int>();
    if (state_timeout_ms_ <= 0 || connect_timeout_ms_ <= 0)
        throw std::runtime_error("BPX timeouts must be positive");
    bpx_fsm::RequireVector(*this, "default_dof_pos");
    for (const char* key : {"fixed_kp", "fixed_kd", "rl_kp", "rl_kd", "action_scale", "torque_limits"})
        for (float value : bpx_fsm::RequireVector(*this, key))
            if (value < 0.0f) throw std::runtime_error(std::string("Negative parameter: ") + key);
    const float passive_kd = params.Get<float>("passive_kd");
    if (!std::isfinite(passive_kd) || passive_kd < 0.0f)
        throw std::runtime_error("Invalid passive damping gain");
    bpx_fsm::Duration(*this, "getup_duration", 2.0f);
    bpx_fsm::Duration(*this, "getdown_duration", 2.0f);
    InitJointNum(12);
    InitObservations();
    InitOutputs();
    InitControl();
    const auto factory_fsm = FSMManager::GetInstance().CreateFSM(robot_name, this);
    if (!factory_fsm) throw std::runtime_error("BPX FSM factory unavailable");
    fsm = *factory_fsm;
}

void RL_Real_Bpx::PreloadPolicy()
{
    const std::string name = params.Get<std::string>("model_name");
    if (name.empty() || name.find('/') != std::string::npos || name.find("..") != std::string::npos)
        throw std::runtime_error("model_name must be a filename in bpx/mirrorme");
    model = InferenceRuntime::ModelFactory::load_model(std::string(POLICY_DIR) + "/bpx/mirrorme/" + name);
    if (!model || !model->is_loaded()) throw std::runtime_error("BPX model preload failed");
    for (int i = 0; i < 2; ++i)
    {
        const auto actions = model->forward({std::vector<float>(45, 0.0f)});
        if (actions.size() != 12) throw std::runtime_error("BPX policy output must contain 12 actions");
        for (float value : actions)
            if (!std::isfinite(value)) throw std::runtime_error("BPX warmup produced nonfinite action");
    }
    obs.actions.assign(12, 0.0f); // warmup results never become robot commands
}

bool RL_Real_Bpx::ReadSnapshot(Snapshot& out)
{
    return ReadHighRateSnapshot(sdk_, out);
}

void RL_Real_Bpx::ConnectAndWait(const char* ip)
{
    sdk_.setRobotIp(ip);
    const int state_port = base_config_["robot_state_port"].as<int>();
    const int joint_port = base_config_["joint_state_port"].as<int>();
    const int state_rate = base_config_["robot_state_rate_hz"].as<int>();
    if (state_port < 1 || state_port > 65535 || joint_port < 1 || joint_port > 65535 ||
        state_port == joint_port || state_rate < 1 || state_rate > 200)
        throw std::runtime_error("Invalid BPX state ports or ordinary upload rate");
    sdk_.setRobotStateUploadPort(static_cast<uint16_t>(state_port));
    sdk_.setJointStateUploadPort(static_cast<uint16_t>(joint_port));
    sdk_.setRobotStateUploadRate(static_cast<uint16_t>(state_rate));
    sdk_.setTcpLocalPort(0);
    if (!sdk_.connect()) throw std::runtime_error("BPX joint connection could not start");
    const auto deadline = Clock::now() + std::chrono::milliseconds(connect_timeout_ms_);
    bool saw_first = false;
    uint32_t first_seq = 0;
    while (!shutdown_requested && Clock::now() < deadline)
    {
        Snapshot candidate;
        if (sdk_.isConnected() && ReadSnapshot(candidate))
        {
            if (!saw_first) { first_seq = candidate.seq; saw_first = true; }
            snapshot_ = candidate;
            have_snapshot_ = true;
            last_new_state_ = Clock::now();
            if (candidate.seq != first_seq)
            {
                GetState(&robot_state);
                std::cout << LOGGER::INFO << "[BPX joint startup] " << SdkStatus(sdk_) << std::endl;
                return;
            }
        }
        else { saw_first = false; have_snapshot_ = false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("BPX TCP/advancing HighRate startup failed; verify JointControl firmware support and DevelopingState. " + SdkStatus(sdk_));
}

void RL_Real_Bpx::GetState(RobotState<float>* state)
{
    if (!sdk_.isConnected()) throw std::runtime_error("BPX control connection lost");
    Snapshot candidate;
    if (ReadSnapshot(candidate))
    {
        if (!have_snapshot_ || candidate.seq != snapshot_.seq)
        {
            if (have_snapshot_ && static_cast<int32_t>(candidate.seq - snapshot_.seq) <= 0)
                throw std::runtime_error("BPX sequence moved backwards; restart explicitly");
            snapshot_ = candidate;
            have_snapshot_ = true;
            last_new_state_ = Clock::now();
        }
    }
    if (!have_snapshot_ || Clock::now() - last_new_state_ > std::chrono::milliseconds(state_timeout_ms_))
        throw std::runtime_error("BPX HighRate telemetry is stale");
    RequireFinite(snapshot_.q, "joint position");
    RequireFinite(snapshot_.dq, "joint velocity");
    RequireFinite(snapshot_.tau, "joint torque");
    RequireFinite(snapshot_.omega, "IMU angular velocity");
    RequireFinite(snapshot_.acc, "IMU acceleration");
    const Quat world_from_imu = Normalize({snapshot_.quat[3], snapshot_.quat[0], snapshot_.quat[1], snapshot_.quat[2]});
    const Quat world_from_body = Normalize(Multiply(world_from_imu, Conjugate(body_from_imu_)));
    Vec3 omega = snapshot_.omega;
    for (float& value : omega) value *= gyro_scale_;
    omega = Rotate(omega_in_world_ ? Conjugate(world_from_body) : body_from_imu_, omega);
    const auto acc = Rotate(body_from_imu_, snapshot_.acc);
    state->imu.quaternion.assign(world_from_body.begin(), world_from_body.end());
    state->imu.gyroscope.assign(omega.begin(), omega.end());
    state->imu.accelerometer.assign(acc.begin(), acc.end());
    for (int p = 0; p < 12; ++p)
    {
        const int s = mapping_[p];
        state->motor_state.q[p] = sign_[p] * (snapshot_.q[s] - offset_[p]);
        state->motor_state.dq[p] = sign_[p] * snapshot_.dq[s];
        state->motor_state.tau_est[p] = sign_[p] * snapshot_.tau[s];
    }
}

void RL_Real_Bpx::SetCommand(const RobotCommand<float>* command)
{
    if (!sdk_.isConnected()) throw std::runtime_error("BPX disconnected before command");
    if (!have_snapshot_ || Clock::now() - last_new_state_ > std::chrono::milliseconds(state_timeout_ms_))
        throw std::runtime_error("BPX feedback expired before command");
    const auto& c = command->motor_command;
    if (c.q.size() != 12 || c.dq.size() != 12 || c.kp.size() != 12 ||
        c.kd.size() != 12 || c.tau.size() != 12)
        throw std::runtime_error("BPX command size mismatch");
    std::array<float, 12> kp{}, pos{}, kd{}, vel{}, tff{};
    for (int p = 0; p < 12; ++p)
    {
        const float values[]{c.q[p], c.dq[p], c.kp[p], c.kd[p], c.tau[p]};
        for (float value : values)
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite BPX command");
        if (c.kp[p] < 0.0f || c.kd[p] < 0.0f)
            throw std::runtime_error("Negative BPX PD gain");
        const int s = mapping_[p];
        kp[s] = c.kp[p];
        kd[s] = c.kd[p];
        pos[s] = offset_[p] + sign_[p] * c.q[p];
        vel[s] = sign_[p] * c.dq[p];
        tff[s] = sign_[p] * c.tau[p];
    }
    RequireFinite(pos, "SDK target position");
    RequireFinite(vel, "SDK target velocity");
    RequireFinite(tff, "SDK feedforward torque");
    if (Clock::now() - last_new_state_ > std::chrono::milliseconds(state_timeout_ms_))
        throw std::runtime_error("BPX feedback expired while preparing command");
    // Same PD command responsibility as Go2: the robot applies joint PD.
    // output_dof_tau is not feedforward; current BPX FSM sends tff=0.
    if (!sdk_.setJointCommand(kp, pos, kd, vel, tff))
        throw std::runtime_error("BPX rejected/failed full joint command");
    command_started_ = true;
}

std::vector<float> RL_Real_Bpx::Forward()
{
    const auto input = ComputeObservation();
    if (input.size() != 45 || !model || !model->is_loaded())
        throw std::runtime_error("BPX policy/input unavailable");
    for (float value : input)
        if (!std::isfinite(value)) throw std::runtime_error("Nonfinite BPX observation");
    auto actions = model->forward({input});
    if (actions.size() != 12) throw std::runtime_error("BPX actor output size mismatch");
    for (float value : actions)
        if (!std::isfinite(value)) throw std::runtime_error("Nonfinite BPX actor output");
    const auto lower = bpx_fsm::RequireVector(*this, "clip_actions_lower");
    const auto upper = bpx_fsm::RequireVector(*this, "clip_actions_upper");
    return clamp(actions, lower, upper);
}

bool RL_Real_Bpx::PolicyActive() const
{
    return rl_init_done && fsm.mode_ == FSM::Mode::NORMAL && fsm.current_state_ &&
        fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion";
}

void RL_Real_Bpx::RunModel()
{
    obs.ang_vel = robot_state.imu.gyroscope;
    obs.base_quat = robot_state.imu.quaternion;
    obs.commands = {control.x, control.y, control.yaw};
    obs.dof_pos = robot_state.motor_state.q;
    obs.dof_vel = robot_state.motor_state.dq;
    const auto actions = Forward(); // reads PREVIOUS obs.actions
    obs.actions = actions;         // previous raw actor action for the next call
    ComputeOutput(actions, output_dof_pos, output_dof_vel, output_dof_tau);
    output_dof_pos_queue.push(output_dof_pos);
    output_dof_vel_queue.push(output_dof_vel);
    // RLControl consumes only pos/vel; no unused tau queue producer.
    ++episode_length_buf;
    const auto state = std::dynamic_pointer_cast<RLFSMState>(fsm.current_state_);
    if (!state) throw std::runtime_error("Unexpected policy state");
    state->RLControl(); // consume fresh target; do not double-run the FSM
}

void RL_Real_Bpx::ResetInputEvents()
{
    control.current_keyboard = Input::Keyboard::None;
    control.last_keyboard = Input::Keyboard::None;
    control.current_gamepad = Input::Gamepad::None;
    control.last_gamepad = Input::Gamepad::None;
}

void RL_Real_Bpx::ForcePassive()
{
    if (fsm.current_state_ && fsm.current_state_->GetStateName() == "RLFSMStatePassive")
    {
        // A held LB+X must cancel pending GetUp without re-entering Passive
        // and printing its Enter message on every 5-ms heartbeat.
        fsm.next_state_ = fsm.current_state_;
        fsm.mode_ = FSM::Mode::NORMAL;
        bpx_fsm::StopPolicyAndDrain(*this);
        InitControl();
        bpx_fsm::Damping(*this, &robot_state, &robot_command);
        policy_phase_ = 0;
        return;
    }
    if (fsm.current_state_) fsm.current_state_->Exit();
    fsm.previous_state_ = fsm.current_state_;
    fsm.mode_ = FSM::Mode::NORMAL;
    fsm.SetInitialState("RLFSMStatePassive");
    bpx_fsm::Damping(*this, &robot_state, &robot_command);
    policy_phase_ = 0;
}

void RL_Real_Bpx::SetupSysJoystick()
{
    const std::string device = "/dev/input/js0";
    if (sys_gamepad_.Setup(device, 16))
        std::cout << LOGGER::INFO << "[BPX joystick] Opened " << device
                  << "; center sticks and release buttons before use" << std::endl;
    else
        std::cout << LOGGER::NOTE << "Joystick [" << device
                  << "] open failed; BPX keyboard remains available" << std::endl;
}

void RL_Real_Bpx::RobotControl()
{
    GetState(&robot_state);
    ResetInputEvents();
    PollKeyboard();
    const auto key = control.current_keyboard;
    ResetInputEvents();
    const bool was_connected = sys_gamepad_.Snapshot().connected;
    sys_gamepad_.Poll(Clock::now()); // shared MuJoCo decoder, internally 10-ms reads
    auto local = sys_gamepad_.Snapshot();
    if (was_connected && !local.connected)
        std::cout << std::endl << LOGGER::NOTE
                  << "[BPX joystick] Disconnected; local velocity cleared. Reconnect and restart to reopen."
                  << std::endl;
    const auto decision = bpx_teleop::DecideMode(key, local);
    control.current_keyboard = decision.keyboard;
    control.current_gamepad = decision.gamepad;
    const auto previous_state = fsm.current_state_;
    if (decision.passive) ForcePassive();
    else StateController(&robot_state, &robot_command);
    const bool changed_state = previous_state != fsm.current_state_;
    // Preserve the mode event above before clearing local readiness for speed.
    // Do not continuously disarm the gamepad in Passive/GetUp: that blocks A.
    if (decision.clear_velocity || changed_state || key == Input::Keyboard::Space)
        sys_gamepad_.RequireNeutral();
    local = sys_gamepad_.Snapshot(); // state only: no second poll/edge consumption
    const bool policy_active = PolicyActive();
    const auto velocity = velocity_input_.Update(
        key, local, decision.clear_velocity || changed_state || !policy_active);
    control.x = velocity.x;
    control.y = velocity.y;
    control.yaw = velocity.yaw;
    // Keep the original entry's last-stage limits before policy inference.
    control.x = std::clamp(control.x, -velocity_limits_[0], velocity_limits_[0]);
    control.y = std::clamp(control.y, -velocity_limits_[1], velocity_limits_[1]);
    control.yaw = std::clamp(control.yaw, -velocity_limits_[2], velocity_limits_[2]);
    if (policy_active)
    {
        if (policy_phase_ == 0) RunModel();
        policy_phase_ = (policy_phase_ + 1) % decimation_;
    }
    else
    {
        bpx_fsm::StopPolicyAndDrain(*this);
        InitControl();
        policy_phase_ = 0;
    }
    ResetInputEvents();
    if (shutdown_requested) return;
    GetState(&robot_state);
    SetCommand(&robot_command);
}

void RL_Real_Bpx::SetupTerminal()
{
    if (!isatty(STDIN_FILENO) || tcgetpgrp(STDIN_FILENO) != getpgrp() ||
        tcgetattr(STDIN_FILENO, &original_terminal_) != 0)
        throw std::runtime_error("BPX keyboard control requires a usable terminal");
    termios terminal = original_terminal_;
    terminal.c_lflag &= ~(ICANON | ECHO);
    terminal.c_cc[VMIN] = 0;
    terminal.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &terminal) != 0)
        throw std::runtime_error("Could not configure terminal");
    terminal_changed_ = true;
}

void RL_Real_Bpx::RestoreTerminal() noexcept
{
    if (terminal_changed_)
    {
        // A background tcsetattr can itself suspend the process (SIGTTOU).
        if (tcgetpgrp(STDIN_FILENO) == getpgrp())
            tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
        terminal_changed_ = false;
    }
}

void RL_Real_Bpx::PollKeyboard()
{
    if (tcgetpgrp(STDIN_FILENO) != getpgrp())
        throw std::runtime_error("BPX control lost its foreground terminal");
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) != 1) return;
    switch (c)
    {
        case '0': control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': control.SetKeyboard(Input::Keyboard::Num1); break;
        case '9': control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'p': case 'P': control.SetKeyboard(Input::Keyboard::P); break;
        case 'w': case 'W': control.SetKeyboard(Input::Keyboard::W); break;
        case 's': case 'S': control.SetKeyboard(Input::Keyboard::S); break;
        case 'a': case 'A': control.SetKeyboard(Input::Keyboard::A); break;
        case 'd': case 'D': control.SetKeyboard(Input::Keyboard::D); break;
        case 'q': case 'Q': control.SetKeyboard(Input::Keyboard::Q); break;
        case 'e': case 'E': control.SetKeyboard(Input::Keyboard::E); break;
        case ' ': control.SetKeyboard(Input::Keyboard::Space); break;
        default: break;
    }
}

void RL_Real_Bpx::StartLoop()
{
    worker_ = std::thread(&RL_Real_Bpx::ControlLoop, this);
}

void RL_Real_Bpx::StopLoop() noexcept
{
    stop_requested_.store(true);
    stop_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void RL_Real_Bpx::ControlLoop() noexcept
{
    const auto period = std::chrono::milliseconds(period_ms_);
    try
    {
        while (!stop_requested_.load() && !shutdown_requested)
        {
            const auto started = Clock::now();
            RobotControl();
            // Nominal 5-ms cadence, without a hard inference-overrun fault.
            std::unique_lock<std::mutex> lock(stop_mutex_);
            stop_cv_.wait_until(lock, started + period, [this] { return stop_requested_.load(); });
        }
    }
    catch (const std::exception& error)
    {
        faulted_.store(true);
        std::cerr << '\n' << LOGGER::ERROR << "BPX fault latched: " << error.what() << std::endl;
    }
    catch (...)
    {
        faulted_.store(true);
        std::cerr << '\n' << LOGGER::ERROR << "BPX fault latched: unknown exception" << std::endl;
    }
    bpx_fsm::StopPolicyAndDrain(*this);
    InitControl();
    BestEffortDamping();
    stop_requested_.store(true);
    stop_cv_.notify_all();
}

void RL_Real_Bpx::BestEffortDamping() noexcept
{
    try
    {
        if (!command_started_ || !sdk_.isConnected()) return;
        std::array<float, 12> kp{}, pos{}, kd{}, vel{}, tff{};
        // With kp=0 the position field has no feedback contribution.
        pos = snapshot_.q;
        for (float& value : pos)
            if (!std::isfinite(value)) value = 0.0f;
        kd.fill(params.Get<float>("passive_kd"));
        sdk_.setJointCommand(kp, pos, kd, vel, tff);
    }
    catch (...) {}
    // No delivery acknowledgement / hardware-stop guarantee is claimed.
}

void RL_Real_Bpx::Close() noexcept
{
    StopLoop(); // object/SDK lifetime outlives every application worker
    for (int i = 0; i < 3; ++i)
    {
        BestEffortDamping();
        if (command_started_) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    try { sdk_.disconnect(); } catch (...) {}
    RestoreTerminal();
}

int main(int argc, char** argv)
{
    const auto usage = [argv](std::ostream& out)
    {
        out << "Usage: " << argv[0]
            << " --monitor ROBOT_IP | --joint-monitor ROBOT_IP | --control ROBOT_IP\n"
            << "--monitor: ordinary StateQuery telemetry, no motor commands\n"
            << "--joint-monitor: JointControl session/HighRate diagnostic, no motor commands; "
               "stop other robot sessions first\n"
            << "--control: local policy and joint PD; uses policy/bpx/base.yaml + bpx/mirrorme/config.yaml\n";
    };
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        usage(std::cout);
        return 0;
    }
    if (argc != 3 || (std::string(argv[1]) != "--monitor" &&
                     std::string(argv[1]) != "--joint-monitor" &&
                     std::string(argv[1]) != "--control"))
    {
        usage(std::cerr);
        return 2;
    }
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    std::signal(SIGTSTP, SignalHandler); // Ctrl+Z requests cleanup instead of suspending control
    try
    {
        const std::string mode = argv[1];
        if (mode == "--monitor") return Monitor(argv[2]);
        if (mode == "--joint-monitor") return JointMonitor(argv[2]);
        RL_Real_Bpx controller(argv[2]);
        while (!shutdown_requested && controller.IsRunning())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return controller.HasFault() ? 1 : 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << '\n' << LOGGER::ERROR << "BPX entry stopped: " << error.what() << std::endl;
        return 1;
    }
}