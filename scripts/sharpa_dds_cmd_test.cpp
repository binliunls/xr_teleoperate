// Build (on Thor, from the unitree_sdk2 source tree in $U):
//   g++ -std=c++17 -O2 -w -I$U/include -I$U/thirdparty/include -I$U/thirdparty/include/ddscxx \
//       sharpa_dds_cmd_test.cpp -o sharpa_dds_cmd_test -L$U/lib/aarch64 -lunitree_sdk2 -L$U/thirdparty/lib/aarch64 \
//       -lddsc -lddscxx -Wl,-rpath,$U/thirdparty/lib/aarch64 -lpthread
//
// Send a small joint command to a Sharpa hand via the DDS bridge and watch the state follow.
// Usage: cmd_test <left|right> <joint_idx> <target_deg> [hold_s]
// Command is in radians (HandCmd_), state is read back in degrees (HandState_).
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/idl/hg/MotorCmd_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace unitree::robot;
using unitree_hg::msg::dds_::HandCmd_;
using unitree_hg::msg::dds_::HandState_;
using unitree_hg::msg::dds_::MotorCmd_;

static constexpr int NUM_JOINTS = 22;
static std::atomic<float> g_q{NAN};
static std::atomic<int> g_joint{0};

int main(int argc, char* argv[]) {
    if (argc < 4) { std::cerr << "usage: " << argv[0] << " <left|right> <joint> <deg> [hold_s]\n"; return 1; }
    const std::string side = argv[1];
    const int joint = std::atoi(argv[2]);
    const float deg = std::atof(argv[3]);
    const int hold_s = argc > 4 ? std::atoi(argv[4]) : 3;
    if (joint < 0 || joint >= NUM_JOINTS || std::fabs(deg) > 20.0f) { std::cerr << "refusing: joint/angle out of safe test range\n"; return 1; }
    g_joint = joint;

    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<HandCmd_> pub("rt/sharpa/" + side + "/cmd");
    pub.InitChannel();
    ChannelSubscriber<HandState_> sub("rt/sharpa/" + side + "/state", [](const void* raw) {
        const HandState_* m = static_cast<const HandState_*>(raw);
        int j = g_joint.load();
        if (static_cast<int>(m->motor_state().size()) > j) g_q = m->motor_state()[j].q();
    });
    sub.InitChannel();
    std::this_thread::sleep_for(std::chrono::seconds(1));   // DDS discovery

    auto send = [&](float rad) {
        HandCmd_ msg;
        std::vector<MotorCmd_> cmds(NUM_JOINTS);
        cmds[joint].q(rad);
        msg.motor_cmd(cmds);
        pub.Write(msg);
    };
    auto phase = [&](const char* name, float rad, int secs) {
        std::cout << "[" << side << "] " << name << ": cmd joint " << joint << " = " << rad * 180.0f / M_PI << " deg\n";
        for (int t = 0; t < secs * 50; ++t) {
            send(rad);
            if (t % 25 == 0) std::cout << "   t=" << t / 50.0 << "s  state[" << joint << "] = " << g_q.load() << " deg\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    };
    phase("baseline", 0.0f, 1);
    phase("move",     deg * M_PI / 180.0f, hold_s);
    phase("return",   0.0f, hold_s);
    std::cout << "[" << side << "] final state[" << joint << "] = " << g_q.load() << " deg\n";
    return 0;
}
