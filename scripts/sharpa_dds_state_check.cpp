// Build (on Thor, from the unitree_sdk2 source tree in $U):
//   g++ -std=c++17 -O2 -w -I$U/include -I$U/thirdparty/include -I$U/thirdparty/include/ddscxx \
//       sharpa_dds_state_check.cpp -o sharpa_dds_state_check -L$U/lib/aarch64 -lunitree_sdk2 -L$U/thirdparty/lib/aarch64 \
//       -lddsc -lddscxx -Wl,-rpath,$U/thirdparty/lib/aarch64 -lpthread
//
// Subscribe to a Sharpa HandState_ topic and report message rate and joint values.
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/HandState_.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace unitree::robot;
using unitree_hg::msg::dds_::HandState_;

static std::atomic<long> g_count{0};
static std::mutex g_mu;
static std::vector<float> g_last;

int main(int argc, char* argv[]) {
    std::string topic = argc > 1 ? argv[1] : "rt/sharpa/left/state";
    int secs = argc > 2 ? std::atoi(argv[2]) : 8;

    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<HandState_> sub(topic, [](const void* raw) {
        const HandState_* m = static_cast<const HandState_*>(raw);
        std::vector<float> q;
        for (const auto& s : m->motor_state()) q.push_back(s.q());
        std::lock_guard<std::mutex> lk(g_mu);
        g_last = std::move(q);
        ++g_count;
    });
    sub.InitChannel();

    std::cout << "subscribed to " << topic << " for " << secs << "s\n";
    long prev = 0;
    for (int i = 1; i <= secs; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        long c = g_count.load();
        std::cout << "t=" << i << "s  msgs/s=" << (c - prev) << "  total=" << c << "\n";
        prev = c;
    }
    std::lock_guard<std::mutex> lk(g_mu);
    std::cout << "last state (" << g_last.size() << " joints, degrees):";
    for (float v : g_last) std::cout << " " << v;
    std::cout << "\n";
    return g_count.load() > 0 ? 0 : 2;
}
