// Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
//
// Flexiv RT zero user-torque drift collector.
// Streams a zero joint torque vector in RT_JOINT_TORQUE and records state drift.

#include <flexiv/rdk/robot.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

constexpr double kPi = 3.14159265358979323846;
std::atomic<bool> g_user_stop{false};

struct Args {
    std::string robot_sn;
    std::string output_dir;
    std::string joint_group = "ARMS";
    double duration_s = 10.0;
    double sample_period_ms = 1.0;
    double drift_limit_deg = 5.0;
    double velocity_limit_rad_s = 0.5;
    double reset_dq_max = 0.02;
    double reset_ddq_max = 0.04;
    double reset_timeout_s = 240.0;
    double reset_tolerance_deg = 0.5;
    bool home_zero = false;
    bool auto_start = false;
    bool enable_gravity_comp = true;
    bool enable_soft_limits = true;
};

struct Sample {
    double time_s = 0.0;
    std::vector<double> command_torque;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> tau_des;
    std::vector<double> tau_ext;
    std::vector<double> theta;
    std::vector<double> dtheta;
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
    std::string out = "\"";
    for (char ch : value) {
        if (ch == '"') {
            out += "\"\"";
        } else {
            out += ch;
        }
    }
    out += "\"";
    return out;
}

double DegToRad(double deg)
{
    return deg * kPi / 180.0;
}

double MaxAbs(const std::vector<double>& values)
{
    double out = 0.0;
    for (double value : values) {
        out = std::max(out, std::abs(value));
    }
    return out;
}

double MaxAbsDiff(const std::vector<double>& a, const std::vector<double>& b)
{
    double out = 0.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        out = std::max(out, std::abs(a[i] - b[i]));
    }
    return out;
}

double SecondsSince(const Clock::time_point& start)
{
    return std::chrono::duration<double>(Clock::now() - start).count();
}

bool ReadFlag(const std::vector<std::string>& argv, const std::string& key)
{
    return std::find(argv.begin(), argv.end(), key) != argv.end();
}

std::string ReadValue(
    const std::vector<std::string>& argv, const std::string& key, const std::string& fallback = "")
{
    auto it = std::find(argv.begin(), argv.end(), key);
    if (it == argv.end()) {
        return fallback;
    }
    ++it;
    if (it == argv.end()) {
        throw std::invalid_argument("Missing value after " + key);
    }
    return *it;
}

double ReadDouble(const std::vector<std::string>& argv, const std::string& key, double fallback)
{
    const auto value = ReadValue(argv, key);
    if (value.empty()) {
        return fallback;
    }
    return std::stod(value);
}

void PrintHelp()
{
    std::cout
        << "Flexiv RT zero user-torque drift collector\n\n"
        << "Required:\n"
        << "  --robot-sn Rizon4s-123456\n"
        << "  --output-dir output/real/flexiv/torque_sysid/zero_drift\n\n"
        << "Options:\n"
        << "  --joint-group ARMS                 Accepted for compatibility; RDK v1.8 uses full-system joints\n"
        << "  --home-zero                        Move slowly to all-zero joints before drift test\n"
        << "  --duration-s 10                    Zero-torque stream duration\n"
        << "  --sample-period-ms 1               Command/sample period\n"
        << "  --drift-limit-deg 5                Stop if any joint drifts this far from home\n"
        << "  --velocity-limit-rad-s 0.5         Stop if any joint exceeds this velocity\n"
        << "  --reset-dq-max 0.02                Home move max velocity\n"
        << "  --reset-ddq-max 0.04               Home move max acceleration\n"
        << "  --reset-timeout-s 240              Home move timeout\n"
        << "  --reset-tolerance-deg 0.5          Required home tolerance before streaming torque\n"
        << "  --disable-gravity-comp             Stream raw zero torques without gravity compensation\n"
        << "  --disable-soft-limits              Disable RDK soft limits in torque command\n"
        << "  --auto-start                       Do not ask for interactive confirmation\n";
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
    out.joint_group = ReadValue(args, "--joint-group", out.joint_group);
    out.duration_s = ReadDouble(args, "--duration-s", out.duration_s);
    out.sample_period_ms = ReadDouble(args, "--sample-period-ms", out.sample_period_ms);
    out.drift_limit_deg = ReadDouble(args, "--drift-limit-deg", out.drift_limit_deg);
    out.velocity_limit_rad_s = ReadDouble(args, "--velocity-limit-rad-s", out.velocity_limit_rad_s);
    out.reset_dq_max = ReadDouble(args, "--reset-dq-max", out.reset_dq_max);
    out.reset_ddq_max = ReadDouble(args, "--reset-ddq-max", out.reset_ddq_max);
    out.reset_timeout_s = ReadDouble(args, "--reset-timeout-s", out.reset_timeout_s);
    out.reset_tolerance_deg = ReadDouble(args, "--reset-tolerance-deg", out.reset_tolerance_deg);
    out.home_zero = ReadFlag(args, "--home-zero");
    out.auto_start = ReadFlag(args, "--auto-start");
    out.enable_gravity_comp = !ReadFlag(args, "--disable-gravity-comp");
    out.enable_soft_limits = !ReadFlag(args, "--disable-soft-limits");

    if (out.robot_sn.empty()) {
        throw std::invalid_argument("--robot-sn is required");
    }
    if (out.output_dir.empty()) {
        throw std::invalid_argument("--output-dir is required");
    }
    if (out.duration_s <= 0.0 || out.sample_period_ms <= 0.0) {
        throw std::invalid_argument("duration and sample period must be positive");
    }
    if (out.drift_limit_deg <= 0.0 || out.velocity_limit_rad_s <= 0.0) {
        throw std::invalid_argument("drift and velocity limits must be positive");
    }
    return out;
}

std::string ResolveGroupName(const std::string& requested)
{
    if (requested.empty() || Upper(requested) == "ARMS" || Upper(requested) == "FULL_SYSTEM") {
        return requested.empty() ? "ARMS" : requested;
    }
    throw std::invalid_argument(
        "RDK v1.8 zero-torque drift collector uses full-system joints only; use --joint-group ARMS");
}

std::vector<double> ReadPosition(flexiv::rdk::Robot& robot)
{
    return robot.states().q;
}

void StreamZeroTorque(flexiv::rdk::Robot& robot, size_t dof, bool gravity_comp, bool soft_limits)
{
    robot.StreamJointTorque(std::vector<double>(dof, 0.0), gravity_comp, soft_limits);
}

void SwitchIdle(flexiv::rdk::Robot& robot)
{
    try {
        robot.Stop();
    } catch (...) {
        try {
            robot.SwitchMode(flexiv::rdk::Mode::IDLE);
        } catch (...) {
        }
    }
}

void MoveToHome(flexiv::rdk::Robot& robot, const Args& args, const std::vector<double>& home_q)
{
    std::cout << "[Flexiv Zero Drift] Moving to home with NRT_JOINT_POSITION" << std::endl;
    robot.SwitchMode(flexiv::rdk::Mode::NRT_JOINT_POSITION);
    std::vector<double> zero(home_q.size(), 0.0);
    std::vector<double> dq_max(home_q.size(), args.reset_dq_max);
    std::vector<double> ddq_max(home_q.size(), args.reset_ddq_max);
    robot.SendJointPosition(home_q, zero, dq_max, ddq_max);

    const auto start = Clock::now();
    const double tolerance_rad = DegToRad(args.reset_tolerance_deg);
    while (SecondsSince(start) < args.reset_timeout_s) {
        if (g_user_stop) {
            throw std::runtime_error("User stop requested during home move");
        }
        const auto q = ReadPosition(robot);
        const double max_error = MaxAbsDiff(q, home_q);
        if (max_error <= tolerance_rad) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const double final_error = MaxAbsDiff(ReadPosition(robot), home_q);
    std::ostringstream msg;
    msg << "Home move timed out with max error " << final_error << " rad ("
        << final_error * 180.0 / kPi << " deg)";
    throw std::runtime_error(msg.str());
}

void WriteJointList(const fs::path& out_dir, size_t dof)
{
    std::ofstream file(out_dir / "joint_list.txt");
    for (size_t i = 0; i < dof; ++i) {
        file << "joint" << (i + 1) << "\n";
    }
}

void WriteControlTorqueCsv(const fs::path& out_dir, const std::vector<Sample>& samples)
{
    std::ofstream file(out_dir / "control_torque.csv");
    file << "type,timestamp,torques\n";
    for (const auto& sample : samples) {
        file << "CONTROL_TORQUE," << FormatDouble(sample.time_s * 1e6) << ","
             << CsvEscape(VecToList(sample.command_torque)) << "\n";
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
    file << "type,timestamp,positions,velocities,torques,tau_des,tau_ext,"
            "motor_positions,motor_velocities,command_torques\n";
    for (const auto& sample : samples) {
        file << "STATE_MOTOR_EXTENDED," << FormatDouble(sample.time_s * 1e6) << ","
             << CsvEscape(VecToList(sample.q)) << "," << CsvEscape(VecToList(sample.dq)) << ","
             << CsvEscape(VecToList(sample.tau)) << "," << CsvEscape(VecToList(sample.tau_des))
             << "," << CsvEscape(VecToList(sample.tau_ext)) << ","
             << CsvEscape(VecToList(sample.theta)) << "," << CsvEscape(VecToList(sample.dtheta))
             << "," << CsvEscape(VecToList(sample.command_torque)) << "\n";
    }
}

void WriteEventCsv(const fs::path& out_dir, const std::string& stop_reason, double duration_s)
{
    std::ofstream file(out_dir / "event.csv");
    file << "type,timestamp,event\n";
    file << "EVENT,0,ZERO_TORQUE_START\n";
    file << "EVENT," << FormatDouble(duration_s * 1e6) << ",ZERO_TORQUE_END_" << stop_reason
         << "\n";
}

void WriteSummaryCsv(const fs::path& out_dir, const std::string& stop_reason, size_t samples,
    double duration_s, double max_abs_drift_rad, double max_abs_velocity_rad_s)
{
    std::ofstream file(out_dir / "drift_summary.csv");
    file << "duration_s,stop_reason,samples,max_abs_drift_rad,max_abs_drift_deg,"
            "max_abs_velocity_rad_s\n";
    file << FormatDouble(duration_s) << "," << CsvEscape(stop_reason) << "," << samples << ","
         << FormatDouble(max_abs_drift_rad) << "," << FormatDouble(max_abs_drift_rad * 180.0 / kPi)
         << "," << FormatDouble(max_abs_velocity_rad_s) << "\n";
}

void WriteMetadataJson(const fs::path& out_dir, const Args& args, const std::string& group_name,
    const std::vector<double>& home_q, const std::string& stop_reason, double max_abs_drift_rad,
    double max_abs_velocity_rad_s)
{
    std::ofstream file(out_dir / "metadata.json");
    file << "{\n";
    file << "  \"robot\": \"flexiv\",\n";
    file << "  \"control_mode\": \"rt_joint_torque_zero_drift\",\n";
    file << "  \"robot_sn\": " << CsvEscape(args.robot_sn) << ",\n";
    file << "  \"joint_group\": " << CsvEscape(group_name) << ",\n";
    file << "  \"home_zero\": " << (args.home_zero ? "true" : "false") << ",\n";
    file << "  \"home_q\": " << VecToList(home_q) << ",\n";
    file << "  \"duration_s\": " << FormatDouble(args.duration_s) << ",\n";
    file << "  \"sample_period_ms\": " << FormatDouble(args.sample_period_ms) << ",\n";
    file << "  \"command_torque_nm\": " << VecToList(std::vector<double>(home_q.size(), 0.0)) << ",\n";
    file << "  \"enable_gravity_comp\": " << (args.enable_gravity_comp ? "true" : "false") << ",\n";
    file << "  \"enable_soft_limits\": " << (args.enable_soft_limits ? "true" : "false") << ",\n";
    file << "  \"drift_limit_deg\": " << FormatDouble(args.drift_limit_deg) << ",\n";
    file << "  \"velocity_limit_rad_s\": " << FormatDouble(args.velocity_limit_rad_s) << ",\n";
    file << "  \"stop_reason\": " << CsvEscape(stop_reason) << ",\n";
    file << "  \"max_abs_drift_rad\": " << FormatDouble(max_abs_drift_rad) << ",\n";
    file << "  \"max_abs_velocity_rad_s\": " << FormatDouble(max_abs_velocity_rad_s) << "\n";
    file << "}\n";
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
        std::cout << "Flexiv RT Zero User-Torque Drift Collector\n";
        std::cout << "Output: " << args.output_dir << "\n";
        std::cout << "Duration: " << args.duration_s << " s\n";
        std::cout << "Drift stop: " << args.drift_limit_deg << " deg\n";
        std::cout << "Velocity stop: " << args.velocity_limit_rad_s << " rad/s\n";
        std::cout << "Gravity comp in zero-torque command: "
                  << (args.enable_gravity_comp ? "on" : "off") << "\n";
        std::cout << "Soft limits in zero-torque command: "
                  << (args.enable_soft_limits ? "on" : "off") << "\n";
        std::cout << "============================================================\n";

        if (!args.auto_start) {
            std::cout << "\nThis will switch to RT_JOINT_TORQUE and stream all-zero user torques.\n"
                      << "With gravity compensation on, this is a gravity-compensated drift test.\n"
                      << "Keep the e-stop ready. Proceed? [y/N]: ";
            std::string response;
            std::getline(std::cin, response);
            if (response != "y" && response != "Y") {
                std::cout << "Cancelled.\n";
                return 1;
            }
        }

        flexiv::rdk::Robot robot(args.robot_sn);
        if (robot.fault()) {
            std::cout << "[Flexiv Zero Drift] Fault detected; attempting ClearFault()\n";
            if (!robot.ClearFault()) {
                throw std::runtime_error("Flexiv fault cannot be cleared");
            }
        }

        std::cout << "[Flexiv Zero Drift] Enabling robot\n";
        robot.Enable();
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        const auto group_name = ResolveGroupName(args.joint_group);
        std::cout << "[Flexiv Zero Drift] Using RDK v1.8 full-system joint API as group: "
                  << group_name << std::endl;

        auto home_q = ReadPosition(robot);
        if (args.home_zero) {
            home_q.assign(home_q.size(), 0.0);
            MoveToHome(robot, args, home_q);
        } else {
            std::cout << "[Flexiv Zero Drift] Using current pose as home" << std::endl;
        }
        std::cout << "[Flexiv Zero Drift] Home q: " << VecToList(home_q) << std::endl;

        std::vector<Sample> samples;
        samples.reserve(static_cast<size_t>(args.duration_s * 1000.0 / args.sample_period_ms) + 1000);
        const std::vector<double> zero_torque(home_q.size(), 0.0);
        const double drift_limit_rad = DegToRad(args.drift_limit_deg);
        double max_abs_drift_rad = 0.0;
        double max_abs_velocity_rad_s = 0.0;
        std::string stop_reason = "timeout";

        robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);
        const auto start = Clock::now();
        auto next_tick = start;
        while (!g_user_stop) {
            const double time_s = SecondsSince(start);
            const auto states = robot.states();
            const double drift = MaxAbsDiff(states.q, home_q);
            const double max_velocity = MaxAbs(states.dq);
            max_abs_drift_rad = std::max(max_abs_drift_rad, drift);
            max_abs_velocity_rad_s = std::max(max_abs_velocity_rad_s, max_velocity);

            if (robot.fault()) {
                stop_reason = "robot_fault";
                break;
            }
            if (drift >= drift_limit_rad) {
                stop_reason = "drift_limit";
                break;
            }
            if (max_velocity >= args.velocity_limit_rad_s) {
                stop_reason = "velocity_limit";
                break;
            }
            if (time_s >= args.duration_s) {
                stop_reason = "timeout";
                break;
            }

            robot.StreamJointTorque(zero_torque, args.enable_gravity_comp, args.enable_soft_limits);

            Sample sample;
            sample.time_s = time_s;
            sample.command_torque = zero_torque;
            sample.q = states.q;
            sample.dq = states.dq;
            sample.tau = states.tau;
            sample.tau_des = states.tau_des;
            sample.tau_ext = states.tau_ext;
            sample.theta = states.theta;
            sample.dtheta = states.dtheta;
            samples.push_back(std::move(sample));

            next_tick += std::chrono::microseconds(static_cast<int>(args.sample_period_ms * 1000.0));
            std::this_thread::sleep_until(next_tick);
            if (Clock::now() > next_tick + std::chrono::milliseconds(10)) {
                next_tick = Clock::now();
            }
        }

        if (g_user_stop) {
            stop_reason = "user_stop";
        }
        try {
            StreamZeroTorque(robot, home_q.size(), args.enable_gravity_comp, args.enable_soft_limits);
        } catch (...) {
        }
        SwitchIdle(robot);

        const double elapsed_s = SecondsSince(start);
        WriteJointList(args.output_dir, home_q.size());
        WriteControlTorqueCsv(args.output_dir, samples);
        WriteStateMotorCsv(args.output_dir, samples);
        WriteExtendedStateCsv(args.output_dir, samples);
        WriteEventCsv(args.output_dir, stop_reason, elapsed_s);
        WriteSummaryCsv(args.output_dir, stop_reason, samples.size(), elapsed_s, max_abs_drift_rad,
            max_abs_velocity_rad_s);
        WriteMetadataJson(args.output_dir, args, group_name, home_q, stop_reason, max_abs_drift_rad,
            max_abs_velocity_rad_s);

        std::cout << "[Flexiv Zero Drift] Stopped: " << stop_reason << " | samples=" << samples.size()
                  << " | max_drift_deg=" << max_abs_drift_rad * 180.0 / kPi
                  << " | max_vel=" << max_abs_velocity_rad_s << std::endl;
        std::cout << "[Flexiv Zero Drift] Data saved to: " << args.output_dir << std::endl;
        return 0;

    } catch (const std::exception& exc) {
        std::cerr << "[Flexiv Zero Drift] ERROR: " << exc.what() << std::endl;
        return 1;
    }
}
