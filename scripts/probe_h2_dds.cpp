/*
 * Read-only H2 DDS topic-rate probe for the robot computer.
 *
 * This program creates subscribers only. It never publishes a command.
 */

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using namespace unitree::robot;
using unitree_hg::msg::dds_::HandState_;
using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;

namespace {

using Clock = std::chrono::steady_clock;
constexpr int kFirstArmJoint = 15;
constexpr int kArmJointCount = 14;

double seconds_now() {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

struct Stats {
    explicit Stats(std::string topic_name) : topic(std::move(topic_name)) {}

    void add(const std::vector<double>& values, double reported_speed = 0.0) {
        const double now = seconds_now();
        std::lock_guard<std::mutex> lock(mutex);
        if (!previous.empty() && previous.size() == values.size()) {
            double step = 0.0;
            for (std::size_t i = 0; i < values.size(); ++i)
                step = std::max(step, std::abs(values[i] - previous[i]));
            max_step = std::max(max_step, step);
            if (step > 1e-7) ++changed;
            if (!arrivals.empty()) {
                const double dt = now - arrivals.back();
                if (dt > 0.0) max_estimated_speed = std::max(max_estimated_speed, step / dt);
            }
        }
        previous = values;
        max_reported_speed = std::max(max_reported_speed, reported_speed);
        arrivals.push_back(now);
    }

    void print(double duration) {
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << std::left << std::setw(25) << topic;
        if (arrivals.empty()) {
            std::cout << "NO DATA in " << duration << "s\n";
            return;
        }
        if (arrivals.size() == 1) {
            std::cout << "one sample only\n";
            return;
        }

        std::vector<double> gaps;
        gaps.reserve(arrivals.size() - 1);
        for (std::size_t i = 1; i < arrivals.size(); ++i)
            gaps.push_back((arrivals[i] - arrivals[i - 1]) * 1000.0);
        std::sort(gaps.begin(), gaps.end());
        const double active_duration = arrivals.back() - arrivals.front();
        const double rate = static_cast<double>(arrivals.size() - 1)
                          / std::max(active_duration, 1e-9);
        const auto percentile = [&](double p) {
            const std::size_t index = std::min(
                gaps.size() - 1,
                static_cast<std::size_t>(std::floor(p * static_cast<double>(gaps.size() - 1)))
            );
            return gaps[index];
        };
        const double change_ratio = 100.0 * static_cast<double>(changed)
                                  / static_cast<double>(arrivals.size() - 1);

        std::cout << std::fixed << std::setprecision(1)
                  << std::right << std::setw(7) << rate << " Hz  "
                  << "gap median/p99/max "
                  << percentile(0.50) << "/" << percentile(0.99) << "/"
                  << gaps.back() << " ms  samples=" << arrivals.size()
                  << "  changed=" << change_ratio << "%"
                  << std::setprecision(4)
                  << "  max_step=" << max_step << " rad"
                  << std::setprecision(2)
                  << "  max_est_speed=" << max_estimated_speed << " rad/s";
        if (max_reported_speed > 0.0)
            std::cout << "  max_reported_dq=" << max_reported_speed << " rad/s";
        std::cout << "\n";
    }

    std::string topic;
    std::mutex mutex;
    std::vector<double> arrivals;
    std::vector<double> previous;
    std::size_t changed{0};
    double max_step{0.0};
    double max_reported_speed{0.0};
    double max_estimated_speed{0.0};
};

Stats lowstate_stats("rt/lowstate");
Stats arm_sdk_stats("rt/arm_sdk");
Stats lowcmd_stats("rt/lowcmd");
Stats left_hand_stats("rt/sharpa/left/state");
Stats right_hand_stats("rt/sharpa/right/state");

void on_lowstate(const void* raw) {
    const auto* message = static_cast<const LowState_*>(raw);
    if (!message || message->motor_state().size() < kFirstArmJoint + kArmJointCount) return;
    std::vector<double> q;
    q.reserve(kArmJointCount);
    double max_dq = 0.0;
    for (int i = kFirstArmJoint; i < kFirstArmJoint + kArmJointCount; ++i) {
        q.push_back(message->motor_state()[i].q());
        max_dq = std::max(max_dq, std::abs(static_cast<double>(message->motor_state()[i].dq())));
    }
    lowstate_stats.add(q, max_dq);
}

void add_lowcmd(const LowCmd_* message, Stats& stats) {
    if (!message || message->motor_cmd().size() < kFirstArmJoint + kArmJointCount) return;
    std::vector<double> q;
    q.reserve(kArmJointCount);
    for (int i = kFirstArmJoint; i < kFirstArmJoint + kArmJointCount; ++i)
        q.push_back(message->motor_cmd()[i].q());
    stats.add(q);
}

void on_arm_sdk(const void* raw) {
    add_lowcmd(static_cast<const LowCmd_*>(raw), arm_sdk_stats);
}

void on_lowcmd(const void* raw) {
    add_lowcmd(static_cast<const LowCmd_*>(raw), lowcmd_stats);
}

void add_hand_state(const HandState_* message, Stats& stats) {
    if (!message || message->motor_state().empty()) return;
    std::vector<double> q;
    q.reserve(message->motor_state().size());
    for (const auto& motor : message->motor_state()) q.push_back(motor.q());
    stats.add(q);
}

void on_left_hand(const void* raw) {
    add_hand_state(static_cast<const HandState_*>(raw), left_hand_stats);
}

void on_right_hand(const void* raw) {
    add_hand_state(static_cast<const HandState_*>(raw), right_hand_stats);
}

}  // namespace

int main(int argc, char** argv) {
    std::string interface;
    int domain = 0;
    double duration = 15.0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) interface = argv[++i];
        else if (arg == "--domain" && i + 1 < argc) domain = std::stoi(argv[++i]);
        else if (arg == "--seconds" && i + 1 < argc) duration = std::stod(argv[++i]);
        else {
            std::cerr << "Usage: " << argv[0]
                      << " --interface IFACE [--domain 0] [--seconds 15]\n";
            return 2;
        }
    }
    if (interface.empty() || duration <= 0.0) {
        std::cerr << "--interface is required and --seconds must be positive\n";
        return 2;
    }

    ChannelFactory::Instance()->Init(domain, interface);

    ChannelSubscriber<LowState_> lowstate_sub("rt/lowstate", on_lowstate);
    ChannelSubscriber<LowCmd_> arm_sdk_sub("rt/arm_sdk", on_arm_sdk);
    ChannelSubscriber<LowCmd_> lowcmd_sub("rt/lowcmd", on_lowcmd);
    ChannelSubscriber<HandState_> left_hand_sub("rt/sharpa/left/state", on_left_hand);
    ChannelSubscriber<HandState_> right_hand_sub("rt/sharpa/right/state", on_right_hand);
    lowstate_sub.InitChannel();
    arm_sdk_sub.InitChannel();
    lowcmd_sub.InitChannel();
    left_hand_sub.InitChannel();
    right_hand_sub.InitChannel();

    std::cout << "Read-only probe: domain=" << domain << ", interface=" << interface
              << ", duration=" << duration << "s\n";
    std::cout << "Start teleop concurrently only if you want to measure rt/arm_sdk.\n";
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    std::cout << "\n";
    lowstate_stats.print(duration);
    arm_sdk_stats.print(duration);
    lowcmd_stats.print(duration);
    left_hand_stats.print(duration);
    right_hand_stats.print(duration);
    return 0;
}
