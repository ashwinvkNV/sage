// Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
//
// Direct Flexiv RT joint-torque SysID collector.
//
// This program deliberately lives outside the Python Flexiv collector because
// the Python RDK package used in the lab does not expose RT_JOINT_TORQUE.

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRtLoopPeriodS = 0.001;

std::atomic<bool> g_user_stop{false};

struct Args {
    std::string robot_sn;
    std::string output_dir;
    std::string joint_group;
    std::string home_plan = "PLAN-Home";
    std::string joints = "all";
    double max_torque_nm = 0.5;
    double offset_limit_deg = 10.0;
    double passive_offset_limit_deg = 3.0;
    double velocity_limit_rad_s = 0.25;
    double trial_timeout_s = 8.0;
    double ramp_duration_s = 2.0;
    double settle_s = 1.0;
    double reset_dq_max = 0.05;
    double reset_ddq_max = 0.10;
    double reset_timeout_s = 45.0;
    double reset_tolerance_deg = 5.0;
    bool enable_gravity_comp = true;
    bool enable_soft_limits = true;
    bool auto_start = false;
};

struct Sample {
    int trial_id = 0;
    int joint_index = 0;
    int sign = 0;
    double time_s = 0.0;
    double trial_time_s = 0.0;
    std::vector<double> command_torque;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> tau_des;
    std::vector<double> tau_ext;
    std::vector<double> theta;
    std::vector<double> dtheta;
};

struct Event {
    double time_s = 0.0;
    std::string name;
};

struct TrialSummary {
    int trial_id = 0;
    int joint_index = 0;
    int sign = 0;
    double start_s = 0.0;
    double end_s = 0.0;
    double max_abs_active_offset_rad = 0.0;
    double max_abs_passive_offset_rad = 0.0;
    double max_abs_velocity_rad_s = 0.0;
    size_t samples = 0;
    std::string stop_reason;
};

struct TrialState {
    flexiv::rdk::Robot* robot = nullptr;
    flexiv::rdk::JointGroup group;
    int trial_id = 0;
    int joint_index = 0;
    int sign = 0;
    size_t dof = 0;
    std::vector<double> home_q;
    double max_torque_nm = 0.0;
    double offset_limit_rad = 0.0;
    double passive_offset_limit_rad = 0.0;
    double velocity_limit_rad_s = 0.0;
    double trial_timeout_s = 0.0;
    double ramp_duration_s = 0.0;
    bool enable_gravity_comp = true;
    bool enable_soft_limits = true;
    Clock::time_point run_start;
    Clock::time_point trial_start;
    std::atomic<bool> stop_requested{false};
    std::string stop_reason;
    double max_abs_active_offset_rad = 0.0;
    double max_abs_passive_offset_rad = 0.0;
    double max_abs_velocity_rad_s = 0.0;
    std::vector<Sample> samples;
};

void HandleSignal(int)
{
    g_user_stop = true;
}

std::string Upper(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

std::vector<std::string> Split(const std::string& value, char delimiter)
{
    std::vector<std::string> items;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        if (!item.empty()) {
            items.push_back(item);
        }
    }
    return items;
}

double DegToRad(double deg)
{
    return deg * kPi / 180.0;
}

double SecondsSince(const Clock::time_point& start)
{
    return std::chrono::duration<double>(Clock::now() - start).count();
}

std::string FormatDouble(double value)
{
    std::ostringstream out;
    out << std::setprecision(12) << value;
    return out.str();
}

std::string VecToList(const std::vector<double>& values)
{
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << FormatDouble(values[i]);
    }
    out << "]";
    return out.str();
}

std::string CsvEscape(const std::string& value)
{
    if (value.find_first_of(",\"\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        } else {
            escaped += c;
        }
    }
    escaped += "\"";
    return escaped;
}

std::string JsonString(const std::string& value)
{
    std::string escaped = "\"";
    for (char c : value) {
        switch (c) {
        case '\\':
            escaped += "\\\\";
            break;
        case '"':
            escaped += "\\\"";
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        default:
            escaped += c;
            break;
        }
    }
    escaped += "\"";
    return escaped;
}

double MaxAbs(const std::vector<double>& values)
{
    double out = 0.0;
    for (double value : values) {
        out = std::max(out, std::abs(value));
    }
    return out;
}

double MaxPassiveOffset(const std::vector<double>& q, const std::vector<double>& home_q, int active_joint)
{
    double out = 0.0;
    for (size_t i = 0; i < q.size(); ++i) {
        if (static_cast<int>(i) == active_joint) {
            continue;
        }
        out = std::max(out, std::abs(q[i] - home_q[i]));
    }
    return out;
}

std::string JointName(int joint_index)
{
    return "joint" + std::to_string(joint_index + 1);
}

void PrintHelp()
{
    std::cout
        << "Flexiv RT joint-torque SysID collector\n\n"
        << "Required:\n"
        << "  --robot-sn Rizon4s-123456\n"
        << "  --output-dir output/real/flexiv/torque_sysid/small_torque\n\n"
        << "Safety defaults are intentionally conservative. Start tiny.\n\n"
        << "Options:\n"
        << "  --joint-group ARMS                 Flexiv joint group, default: first group\n"
        << "  --home-plan PLAN-Home              Plan used before each torque trial, default: PLAN-Home\n"
        << "  --no-home-plan                     Reset by slow NRT joint position to the recorded home pose\n"
        << "  --joints all|joint1,joint2|1,2     Joints to probe, default: all\n"
        << "  --max-torque-nm 0.5                Peak commanded torque on active joint\n"
        << "  --offset-limit-deg 10              Stop active joint after this home offset\n"
        << "  --passive-offset-limit-deg 3       Stop if any inactive joint moves this much\n"
        << "  --velocity-limit-rad-s 0.25        Stop if any joint velocity exceeds this\n"
        << "  --trial-timeout-s 8                Stop a trial after this duration\n"
        << "  --ramp-duration-s 2                Smooth half-cosine torque ramp duration\n"
        << "  --settle-s 1                       Settle time after reset\n"
        << "  --reset-dq-max 0.05                NRT reset max velocity if --no-home-plan\n"
        << "  --reset-ddq-max 0.10               NRT reset max acceleration if --no-home-plan\n"
        << "  --disable-gravity-comp             Send RtJointTorqueCmd with gravity compensation disabled\n"
        << "  --disable-soft-limits              Send RtJointTorqueCmd with soft limits disabled\n"
        << "  --auto-start                       Do not ask for interactive confirmation\n";
}

bool ReadFlag(const std::vector<std::string>& argv, const std::string& key)
{
    return std::find(argv.begin(), argv.end(), key) != argv.end();
}

std::string ReadValue(const std::vector<std::string>& argv, const std::string& key, const std::string& fallback = "")
{
    for (size_t i = 0; i + 1 < argv.size(); ++i) {
        if (argv[i] == key) {
            return argv[i + 1];
        }
    }
    return fallback;
}

double ReadDouble(const std::vector<std::string>& argv, const std::string& key, double fallback)
{
    const auto value = ReadValue(argv, key);
    if (value.empty()) {
        return fallback;
    }
    return std::stod(value);
}

Args ParseArgs(int argc, char* argv[])
{
    std::vector<std::string> args(argv + 1, argv + argc);
    if (ReadFlag(args, "-h") || ReadFlag(args, "--help")) {
        PrintHelp();
        std::exit(0);
    }

    Args out;
    out.robot_sn = ReadValue(args, "--robot-sn");
    out.output_dir = ReadValue(args, "--output-dir");
    out.joint_group = ReadValue(args, "--joint-group");
    out.home_plan = ReadValue(args, "--home-plan", out.home_plan);
    out.joints = ReadValue(args, "--joints", out.joints);
    out.max_torque_nm = ReadDouble(args, "--max-torque-nm", out.max_torque_nm);
    out.offset_limit_deg = ReadDouble(args, "--offset-limit-deg", out.offset_limit_deg);
    out.passive_offset_limit_deg =
        ReadDouble(args, "--passive-offset-limit-deg", out.passive_offset_limit_deg);
    out.velocity_limit_rad_s = ReadDouble(args, "--velocity-limit-rad-s", out.velocity_limit_rad_s);
    out.trial_timeout_s = ReadDouble(args, "--trial-timeout-s", out.trial_timeout_s);
    out.ramp_duration_s = ReadDouble(args, "--ramp-duration-s", out.ramp_duration_s);
    out.settle_s = ReadDouble(args, "--settle-s", out.settle_s);
    out.reset_dq_max = ReadDouble(args, "--reset-dq-max", out.reset_dq_max);
    out.reset_ddq_max = ReadDouble(args, "--reset-ddq-max", out.reset_ddq_max);
    out.reset_timeout_s = ReadDouble(args, "--reset-timeout-s", out.reset_timeout_s);
    out.reset_tolerance_deg = ReadDouble(args, "--reset-tolerance-deg", out.reset_tolerance_deg);
    out.enable_gravity_comp = !ReadFlag(args, "--disable-gravity-comp");
    out.enable_soft_limits = !ReadFlag(args, "--disable-soft-limits");
    out.auto_start = ReadFlag(args, "--auto-start");
    if (ReadFlag(args, "--no-home-plan")) {
        out.home_plan.clear();
    }

    if (out.robot_sn.empty()) {
        throw std::invalid_argument("--robot-sn is required");
    }
    if (out.output_dir.empty()) {
        throw std::invalid_argument("--output-dir is required");
    }
    if (out.max_torque_nm <= 0.0) {
        throw std::invalid_argument("--max-torque-nm must be positive");
    }
    if (out.offset_limit_deg <= 0.0 || out.passive_offset_limit_deg <= 0.0) {
        throw std::invalid_argument("offset limits must be positive");
    }
    if (out.velocity_limit_rad_s <= 0.0 || out.trial_timeout_s <= 0.0 || out.ramp_duration_s <= 0.0) {
        throw std::invalid_argument("velocity, timeout, and ramp limits must be positive");
    }
    return out;
}

flexiv::rdk::JointGroup ResolveGroup(flexiv::rdk::Robot& robot, const std::string& requested)
{
    const auto groups = robot.groups();
    if (groups.empty()) {
        throw std::runtime_error("Robot reported no joint groups");
    }
    if (requested.empty()) {
        return groups.front();
    }

    const auto requested_upper = Upper(requested);
    for (const auto& group : groups) {
        const auto name = flexiv::rdk::kJointGroupNames.at(group);
        if (Upper(name) == requested_upper) {
            return group;
        }
    }

    std::ostringstream msg;
    msg << "Joint group '" << requested << "' not found. Available groups:";
    for (const auto& group : groups) {
        msg << " " << flexiv::rdk::kJointGroupNames.at(group);
    }
    throw std::runtime_error(msg.str());
}

std::vector<int> ResolveJointIndices(const std::string& spec, size_t dof)
{
    if (spec.empty() || Upper(spec) == "ALL") {
        std::vector<int> out(dof);
        for (size_t i = 0; i < dof; ++i) {
            out[i] = static_cast<int>(i);
        }
        return out;
    }

    std::vector<int> out;
    for (auto token : Split(spec, ',')) {
        token = Upper(token);
        if (token.rfind("JOINT", 0) == 0) {
            token = token.substr(5);
        }
        const int one_based = std::stoi(token);
        const int idx = one_based - 1;
        if (idx < 0 || idx >= static_cast<int>(dof)) {
            throw std::invalid_argument("Joint index out of range in --joints: " + token);
        }
        out.push_back(idx);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

void StreamZeroTorque(TrialState& state)
{
    std::vector<double> zero(state.dof, 0.0);
    std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointTorqueCmd> cmds;
    cmds[state.group] =
        flexiv::rdk::RtJointTorqueCmd(zero, state.enable_gravity_comp, state.enable_soft_limits);
    state.robot->StreamJointTorque(cmds);
}

void RequestStop(TrialState& state, const std::string& reason)
{
    if (!state.stop_requested.exchange(true)) {
        state.stop_reason = reason;
    }
    try {
        StreamZeroTorque(state);
    } catch (...) {
    }
}

double SmoothRamp(double t, double duration)
{
    if (t >= duration) {
        return 1.0;
    }
    const double alpha = std::max(0.0, std::min(1.0, t / duration));
    return 0.5 - 0.5 * std::cos(kPi * alpha);
}

void TorquePeriodicTask(TrialState& state)
{
    try {
        if (g_user_stop) {
            RequestStop(state, "user_stop");
            return;
        }
        if (state.robot->fault()) {
            RequestStop(state, "robot_fault");
            return;
        }

        const double time_s = SecondsSince(state.run_start);
        const double trial_time_s = SecondsSince(state.trial_start);
        const auto all_states = state.robot->states();
        const auto states_it = all_states.find(state.group);
        if (states_it == all_states.end()) {
            RequestStop(state, "missing_joint_group_state");
            return;
        }
        const auto& robot_states = states_it->second;
        if (robot_states.q.size() != state.dof || robot_states.dq.size() != state.dof) {
            RequestStop(state, "state_size_mismatch");
            return;
        }

        const double active_offset = robot_states.q[state.joint_index] - state.home_q[state.joint_index];
        const double max_passive_offset =
            MaxPassiveOffset(robot_states.q, state.home_q, state.joint_index);
        const double max_velocity = MaxAbs(robot_states.dq);
        state.max_abs_active_offset_rad =
            std::max(state.max_abs_active_offset_rad, std::abs(active_offset));
        state.max_abs_passive_offset_rad =
            std::max(state.max_abs_passive_offset_rad, max_passive_offset);
        state.max_abs_velocity_rad_s = std::max(state.max_abs_velocity_rad_s, max_velocity);

        if (std::abs(active_offset) >= state.offset_limit_rad) {
            RequestStop(state, "active_offset_limit");
            return;
        }
        if (max_passive_offset >= state.passive_offset_limit_rad) {
            RequestStop(state, "passive_offset_limit");
            return;
        }
        if (max_velocity >= state.velocity_limit_rad_s) {
            RequestStop(state, "velocity_limit");
            return;
        }
        if (trial_time_s >= state.trial_timeout_s) {
            RequestStop(state, "timeout");
            return;
        }

        std::vector<double> command_torque(state.dof, 0.0);
        command_torque[state.joint_index] =
            state.sign * state.max_torque_nm * SmoothRamp(trial_time_s, state.ramp_duration_s);

        std::map<flexiv::rdk::JointGroup, flexiv::rdk::RtJointTorqueCmd> cmds;
        cmds[state.group] = flexiv::rdk::RtJointTorqueCmd(
            command_torque, state.enable_gravity_comp, state.enable_soft_limits);
        state.robot->StreamJointTorque(cmds);

        Sample sample;
        sample.trial_id = state.trial_id;
        sample.joint_index = state.joint_index;
        sample.sign = state.sign;
        sample.time_s = time_s;
        sample.trial_time_s = trial_time_s;
        sample.command_torque = command_torque;
        sample.q = robot_states.q;
        sample.dq = robot_states.dq;
        sample.tau = robot_states.tau;
        sample.tau_des = robot_states.tau_des;
        sample.tau_ext = robot_states.tau_ext;
        sample.theta = robot_states.theta;
        sample.dtheta = robot_states.dtheta;
        state.samples.push_back(std::move(sample));

    } catch (const std::exception& exc) {
        RequestStop(state, std::string("exception:") + exc.what());
    } catch (...) {
        RequestStop(state, "unknown_exception");
    }
}

std::vector<double> ReadGroupPosition(flexiv::rdk::Robot& robot, flexiv::rdk::JointGroup group)
{
    const auto states = robot.states();
    const auto it = states.find(group);
    if (it == states.end()) {
        throw std::runtime_error("Could not read selected joint group state");
    }
    return it->second.q;
}

void WaitUntilNotBusy(flexiv::rdk::Robot& robot)
{
    while (robot.busy()) {
        if (g_user_stop) {
            throw std::runtime_error("User stop requested while waiting for robot");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ResetToHome(flexiv::rdk::Robot& robot, flexiv::rdk::JointGroup group, const Args& args,
    const std::vector<double>& home_q)
{
    if (!args.home_plan.empty()) {
        std::cout << "[Flexiv RT Torque] Executing reset plan: " << args.home_plan << std::endl;
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan(args.home_plan);
        WaitUntilNotBusy(robot);
    } else {
        std::cout << "[Flexiv RT Torque] Resetting to recorded home with NRT_JOINT_POSITION"
                  << std::endl;
        robot.SwitchMode(flexiv::rdk::Mode::NRT_JOINT_POSITION);
        std::vector<double> zero(home_q.size(), 0.0);
        std::vector<double> dq_max(home_q.size(), args.reset_dq_max);
        std::vector<double> ddq_max(home_q.size(), args.reset_ddq_max);
        std::map<flexiv::rdk::JointGroup, flexiv::rdk::NrtJointPositionCmd> cmds;
        cmds[group] = flexiv::rdk::NrtJointPositionCmd(home_q, zero, dq_max, ddq_max);
        robot.SendJointPosition(cmds);

        const auto start = Clock::now();
        while (SecondsSince(start) < args.reset_timeout_s) {
            if (g_user_stop) {
                throw std::runtime_error("User stop requested during NRT reset");
            }
            const auto stopped = robot.stopped();
            const auto it = stopped.find(group);
            if (it != stopped.end() && it->second) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    if (args.settle_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(args.settle_s));
    }
}

TrialSummary RunTrial(flexiv::rdk::Robot& robot, flexiv::rdk::JointGroup group, const Args& args,
    const std::vector<double>& home_q, int trial_id, int joint_index, int sign,
    const Clock::time_point& run_start, std::vector<Sample>& all_samples)
{
    std::cout << "[Flexiv RT Torque] Trial " << trial_id << ": " << JointName(joint_index)
              << (sign > 0 ? " positive" : " negative") << " torque" << std::endl;

    robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

    TrialState state;
    state.robot = &robot;
    state.group = group;
    state.trial_id = trial_id;
    state.joint_index = joint_index;
    state.sign = sign;
    state.dof = home_q.size();
    state.home_q = home_q;
    state.max_torque_nm = args.max_torque_nm;
    state.offset_limit_rad = DegToRad(args.offset_limit_deg);
    state.passive_offset_limit_rad = DegToRad(args.passive_offset_limit_deg);
    state.velocity_limit_rad_s = args.velocity_limit_rad_s;
    state.trial_timeout_s = args.trial_timeout_s;
    state.ramp_duration_s = args.ramp_duration_s;
    state.enable_gravity_comp = args.enable_gravity_comp;
    state.enable_soft_limits = args.enable_soft_limits;
    state.run_start = run_start;
    state.trial_start = Clock::now();
    state.samples.reserve(static_cast<size_t>(args.trial_timeout_s / kRtLoopPeriodS) + 1000);

    flexiv::rdk::Scheduler scheduler;
    scheduler.AddTask(std::bind(TorquePeriodicTask, std::ref(state)), "RT torque sysid", 1,
        scheduler.max_priority());
    scheduler.Start();
    while (!state.stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    scheduler.Stop();

    try {
        StreamZeroTorque(state);
    } catch (...) {
    }

    const double end_s = SecondsSince(run_start);
    try {
        robot.Stop();
    } catch (const std::exception& exc) {
        std::cerr << "[Flexiv RT Torque] Warning: robot.Stop() failed: " << exc.what()
                  << std::endl;
    }

    TrialSummary summary;
    summary.trial_id = trial_id;
    summary.joint_index = joint_index;
    summary.sign = sign;
    summary.start_s = std::chrono::duration<double>(state.trial_start - run_start).count();
    summary.end_s = end_s;
    summary.max_abs_active_offset_rad = state.max_abs_active_offset_rad;
    summary.max_abs_passive_offset_rad = state.max_abs_passive_offset_rad;
    summary.max_abs_velocity_rad_s = state.max_abs_velocity_rad_s;
    summary.samples = state.samples.size();
    summary.stop_reason = state.stop_reason.empty() ? "unknown" : state.stop_reason;

    all_samples.insert(all_samples.end(), state.samples.begin(), state.samples.end());

    std::cout << "[Flexiv RT Torque] Trial " << trial_id << " stopped: " << summary.stop_reason
              << " | samples=" << summary.samples
              << " | active_offset_deg=" << summary.max_abs_active_offset_rad * 180.0 / kPi
              << " | passive_offset_deg=" << summary.max_abs_passive_offset_rad * 180.0 / kPi
              << " | max_vel=" << summary.max_abs_velocity_rad_s << std::endl;
    return summary;
}

void WriteJointList(const fs::path& out_dir, size_t dof)
{
    std::ofstream file(out_dir / "joint_list.txt");
    for (size_t i = 0; i < dof; ++i) {
        file << JointName(static_cast<int>(i)) << "\n";
    }
}

void WriteControlTorqueCsv(const fs::path& out_dir, const std::vector<Sample>& samples)
{
    std::ofstream file(out_dir / "control_torque.csv");
    file << "type,timestamp,trial_id,active_joint,sign,torques\n";
    for (const auto& sample : samples) {
        file << "CONTROL_TORQUE," << FormatDouble(sample.time_s * 1e6) << ","
             << sample.trial_id << "," << JointName(sample.joint_index) << "," << sample.sign
             << "," << CsvEscape(VecToList(sample.command_torque)) << "\n";
    }
}

void WriteStateMotorCsv(const fs::path& out_dir, const std::vector<Sample>& samples)
{
    std::ofstream file(out_dir / "state_motor.csv");
    file << "type,timestamp,positions,velocities,torques\n";
    for (const auto& sample : samples) {
        file << "STATE_MOTOR," << FormatDouble(sample.time_s * 1e6) << ","
             << CsvEscape(VecToList(sample.q)) << "," << CsvEscape(VecToList(sample.dq)) << ","
             << CsvEscape(VecToList(sample.tau)) << "\n";
    }
}

void WriteExtendedStateCsv(const fs::path& out_dir, const std::vector<Sample>& samples)
{
    std::ofstream file(out_dir / "state_motor_extended.csv");
    file << "type,timestamp,trial_id,active_joint,sign,trial_time_s,positions,velocities,"
            "torques,tau_des,tau_ext,motor_positions,motor_velocities,command_torques\n";
    for (const auto& sample : samples) {
        file << "STATE_MOTOR_EXTENDED," << FormatDouble(sample.time_s * 1e6) << ","
             << sample.trial_id << "," << JointName(sample.joint_index) << "," << sample.sign
             << "," << FormatDouble(sample.trial_time_s) << "," << CsvEscape(VecToList(sample.q))
             << "," << CsvEscape(VecToList(sample.dq)) << "," << CsvEscape(VecToList(sample.tau))
             << "," << CsvEscape(VecToList(sample.tau_des)) << ","
             << CsvEscape(VecToList(sample.tau_ext)) << "," << CsvEscape(VecToList(sample.theta))
             << "," << CsvEscape(VecToList(sample.dtheta)) << ","
             << CsvEscape(VecToList(sample.command_torque)) << "\n";
    }
}

void WriteEventCsv(const fs::path& out_dir, const std::vector<Event>& events)
{
    std::ofstream file(out_dir / "event.csv");
    file << "type,timestamp,event\n";
    for (const auto& event : events) {
        file << "EVENT," << FormatDouble(event.time_s * 1e6) << "," << CsvEscape(event.name)
             << "\n";
    }
}

void WriteTrialSummaryCsv(const fs::path& out_dir, const std::vector<TrialSummary>& summaries)
{
    std::ofstream file(out_dir / "trial_summary.csv");
    file << "trial_id,active_joint,sign,start_s,end_s,duration_s,stop_reason,samples,"
            "max_abs_active_offset_rad,max_abs_passive_offset_rad,max_abs_velocity_rad_s\n";
    for (const auto& summary : summaries) {
        file << summary.trial_id << "," << JointName(summary.joint_index) << "," << summary.sign
             << "," << FormatDouble(summary.start_s) << "," << FormatDouble(summary.end_s) << ","
             << FormatDouble(summary.end_s - summary.start_s) << ","
             << CsvEscape(summary.stop_reason) << "," << summary.samples << ","
             << FormatDouble(summary.max_abs_active_offset_rad) << ","
             << FormatDouble(summary.max_abs_passive_offset_rad) << ","
             << FormatDouble(summary.max_abs_velocity_rad_s) << "\n";
    }
}

void WriteMetadataJson(const fs::path& out_dir, const Args& args, const std::string& group_name,
    const std::vector<double>& home_q, const std::vector<int>& joints)
{
    std::ofstream file(out_dir / "metadata.json");
    file << "{\n";
    file << "  \"robot\": \"flexiv\",\n";
    file << "  \"control_mode\": \"rt_joint_torque\",\n";
    file << "  \"robot_sn\": " << JsonString(args.robot_sn) << ",\n";
    file << "  \"joint_group\": " << JsonString(group_name) << ",\n";
    file << "  \"home_plan\": " << JsonString(args.home_plan) << ",\n";
    file << "  \"home_q\": " << VecToList(home_q) << ",\n";
    file << "  \"joints\": [";
    for (size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) {
            file << ", ";
        }
        file << "\"" << JointName(joints[i]) << "\"";
    }
    file << "],\n";
    file << "  \"max_torque_nm\": " << FormatDouble(args.max_torque_nm) << ",\n";
    file << "  \"offset_limit_deg\": " << FormatDouble(args.offset_limit_deg) << ",\n";
    file << "  \"passive_offset_limit_deg\": " << FormatDouble(args.passive_offset_limit_deg)
         << ",\n";
    file << "  \"velocity_limit_rad_s\": " << FormatDouble(args.velocity_limit_rad_s) << ",\n";
    file << "  \"trial_timeout_s\": " << FormatDouble(args.trial_timeout_s) << ",\n";
    file << "  \"ramp_duration_s\": " << FormatDouble(args.ramp_duration_s) << ",\n";
    file << "  \"enable_gravity_comp\": " << (args.enable_gravity_comp ? "true" : "false")
         << ",\n";
    file << "  \"enable_soft_limits\": " << (args.enable_soft_limits ? "true" : "false")
         << ",\n";
    file << "  \"timestamp_policy\": {\n";
    file << "    \"control_torque_csv_timestamp\": \"command_stream_time_s\",\n";
    file << "    \"state_motor_csv_timestamp\": \"state_read_and_command_stream_loop_time_s\",\n";
    file << "    \"units\": \"microseconds_from_collection_start\"\n";
    file << "  }\n";
    file << "}\n";
}

void ValidateResetPosition(
    flexiv::rdk::Robot& robot, flexiv::rdk::JointGroup group, const Args& args,
    const std::vector<double>& home_q)
{
    const auto q = ReadGroupPosition(robot, group);
    double max_offset = 0.0;
    for (size_t i = 0; i < q.size(); ++i) {
        max_offset = std::max(max_offset, std::abs(q[i] - home_q[i]));
    }
    const double max_offset_deg = max_offset * 180.0 / kPi;
    if (max_offset_deg > args.reset_tolerance_deg) {
        std::ostringstream msg;
        msg << "Reset ended " << max_offset_deg << " deg from recorded home, above tolerance "
            << args.reset_tolerance_deg << " deg";
        throw std::runtime_error(msg.str());
    }
}

} // namespace

int main(int argc, char* argv[])
{
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    try {
        const Args args = ParseArgs(argc, argv);
        fs::create_directories(args.output_dir);

        std::cout << "============================================================\n";
        std::cout << "Flexiv RT Joint Torque SysID Collector\n";
        std::cout << "Output: " << args.output_dir << "\n";
        std::cout << "Peak torque: " << args.max_torque_nm << " Nm\n";
        std::cout << "Active offset stop: " << args.offset_limit_deg << " deg\n";
        std::cout << "Passive offset stop: " << args.passive_offset_limit_deg << " deg\n";
        std::cout << "Velocity stop: " << args.velocity_limit_rad_s << " rad/s\n";
        std::cout << "Gravity comp in torque command: "
                  << (args.enable_gravity_comp ? "on" : "off") << "\n";
        std::cout << "Soft limits in torque command: " << (args.enable_soft_limits ? "on" : "off")
                  << "\n";
        std::cout << "============================================================\n";

        if (!args.auto_start) {
            std::cout << "\nThis will stream direct joint torque commands at 1 kHz.\n"
                      << "Start with a very small --max-torque-nm and keep the e-stop ready.\n"
                      << "Proceed? [y/N]: ";
            std::string response;
            std::getline(std::cin, response);
            if (response != "y" && response != "Y") {
                std::cout << "Cancelled.\n";
                return 1;
            }
        }

        flexiv::rdk::Robot robot(args.robot_sn);
        if (robot.fault()) {
            std::cout << "[Flexiv RT Torque] Fault detected; attempting ClearFault()\n";
            if (!robot.ClearFault()) {
                throw std::runtime_error("Flexiv fault cannot be cleared");
            }
        }

        std::cout << "[Flexiv RT Torque] Enabling robot\n";
        robot.Enable();
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        const auto group = ResolveGroup(robot, args.joint_group);
        const auto group_name = flexiv::rdk::kJointGroupNames.at(group);
        std::cout << "[Flexiv RT Torque] Using joint group: " << group_name << std::endl;

        if (!args.home_plan.empty()) {
            ResetToHome(robot, group, args, {});
        } else {
            std::cout << "[Flexiv RT Torque] No home plan configured; using current pose as home"
                      << std::endl;
        }
        const auto home_q = ReadGroupPosition(robot, group);
        const auto joints = ResolveJointIndices(args.joints, home_q.size());
        std::cout << "[Flexiv RT Torque] Recorded home q: " << VecToList(home_q) << std::endl;

        WriteJointList(args.output_dir, home_q.size());

        std::vector<Sample> all_samples;
        std::vector<TrialSummary> summaries;
        std::vector<Event> events;
        const auto run_start = Clock::now();
        events.push_back({0.0, "CONTROL_MODE_ENABLE"});

        int trial_id = 0;
        for (int joint_index : joints) {
            for (int sign : {1, -1}) {
                if (g_user_stop) {
                    throw std::runtime_error("User stop requested before next trial");
                }
                ResetToHome(robot, group, args, home_q);
                ValidateResetPosition(robot, group, args, home_q);
                const double start_s = SecondsSince(run_start);
                events.push_back(
                    {start_s, "TRIAL_START_" + JointName(joint_index) + (sign > 0 ? "_pos" : "_neg")});
                auto summary =
                    RunTrial(robot, group, args, home_q, ++trial_id, joint_index, sign, run_start,
                        all_samples);
                summaries.push_back(summary);
                events.push_back({summary.end_s,
                    "TRIAL_END_" + JointName(joint_index) + (sign > 0 ? "_pos_" : "_neg_")
                        + summary.stop_reason});
            }
        }
        events.push_back({SecondsSince(run_start), "MOTION_END"});

        WriteControlTorqueCsv(args.output_dir, all_samples);
        WriteStateMotorCsv(args.output_dir, all_samples);
        WriteExtendedStateCsv(args.output_dir, all_samples);
        WriteEventCsv(args.output_dir, events);
        WriteTrialSummaryCsv(args.output_dir, summaries);
        WriteMetadataJson(args.output_dir, args, group_name, home_q, joints);

        std::cout << "\n[Flexiv RT Torque] Collection complete.\n";
        std::cout << "[Flexiv RT Torque] Data saved to: " << args.output_dir << "\n";
        return 0;

    } catch (const std::exception& exc) {
        std::cerr << "[Flexiv RT Torque] ERROR: " << exc.what() << std::endl;
        return 1;
    }
}
