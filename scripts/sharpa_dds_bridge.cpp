/*
 * Sharpa hand DDS bridge — run on Thor (aarch64).
 *
 * Connects to both Sharpa hands via the C++ SDK and bridges them to DDS so the
 * workstation can command and read state without the SDK installed.
 *
 * DDS topics:
 *   Subscribe  rt/sharpa/left/cmd    HandCmd_   motor_cmd[i].q() in radians
 *   Subscribe  rt/sharpa/right/cmd   HandCmd_   motor_cmd[i].q() in radians
 *   Publish    rt/sharpa/left/state  HandState_ motor_state[i].q() in degrees
 *   Publish    rt/sharpa/right/state HandState_ motor_state[i].q() in degrees
 *
 * Usage:
 *   ./sharpa_dds_bridge [--speed 0.5] [--state-hz 50]
 */

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/idl/hg/MotorCmd_.hpp>
#include <unitree/idl/hg/MotorState_.hpp>

#include <SharpaWaveSDK.h>

#include <zmq.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

static constexpr int  NUM_JOINTS = 22;
static constexpr int  CMD_HZ     = 50;   // command apply rate
static constexpr char TOPIC_LEFT_CMD[]    = "rt/sharpa/left/cmd";
static constexpr char TOPIC_RIGHT_CMD[]   = "rt/sharpa/right/cmd";
static constexpr char TOPIC_LEFT_STATE[]  = "rt/sharpa/left/state";
static constexpr char TOPIC_RIGHT_STATE[] = "rt/sharpa/right/state";

using namespace unitree::robot;
using unitree_hg::msg::dds_::HandCmd_;
using unitree_hg::msg::dds_::HandState_;
using unitree_hg::msg::dds_::MotorState_;

// Latest per-hand joint angles (DEGREES), refreshed by each HandBridge's
// state_loop and read by the tactile publisher so every fingertip frame
// ships with the contemporaneous hand pose. NUM_JOINTS matches the Python
// wire's JOINTS_LEN (22). Declared before TactilePublisher so publish() sees it.
struct JointCache {
    std::mutex mu;
    std::array<float, NUM_JOINTS> left{};
    std::array<float, NUM_JOINTS> right{};
    bool left_valid  = false;
    bool right_valid = false;
};
static JointCache g_joints;

// ── tactile ZMQ publisher ─────────────────────────────────────────────────────
//
// One PUB socket shared by all hands. Per fingertip frame, packs DEFORM + F6 +
// CONTACT_POINT into a 28-byte little-endian header + raw payload and sends as
// one ZMQ message. RAW is intentionally dropped — the workstation recorder
// doesn't need it and it would ~3× the bandwidth.
//
// Wire format mirrors teleop/utils/sharpa_tactile_wire.py:
//   u32 channel | u32 frame_id | f64 ts | u32 dlen | u32 flen | u32 cplen | bytes...
//
// SDK fires tactile callbacks from its own thread(s); 2 hands × 5 channels can
// race on the socket, so send() is mutex-guarded. ZMQ sockets are NOT
// thread-safe on their own.

class TactilePublisher {
public:
    explicit TactilePublisher(int port)
        : ctx_(1), sock_(ctx_, zmq::socket_type::pub) {
        sock_.set(zmq::sockopt::sndhwm, 100);
        sock_.set(zmq::sockopt::linger, 0);
        const std::string addr = "tcp://*:" + std::to_string(port);
        sock_.bind(addr);
        std::cout << "[tactile] PUB " << addr << " ready\n";
    }

    void publish(sharpa::tactile::Frame::Ptr fr) {
        if (!fr) return;
        auto find_block = [&](const char* key) -> sharpa::tactile::DataBlock::Ptr {
            auto it = fr->content.find(key);
            return (it != fr->content.end()) ? it->second : nullptr;
        };
        auto d_blk  = find_block("DEFORM");
        auto f_blk  = find_block("F6");
        auto cp_blk = find_block("CONTACT_POINT");

        const size_t d_len  = (d_blk  && d_blk->data())  ? d_blk->nbytes()  : 0;
        const size_t f_len  = (f_blk  && f_blk->data())  ? f_blk->nbytes()  : 0;
        const size_t cp_len = (cp_blk && cp_blk->data()) ? cp_blk->nbytes() : 0;

        // Snapshot the owning hand's latest joints (DEGREES). channel >=5 = left,
        // <5 = right (matches channel_to_hand_finger). Appended as an optional
        // fixed-size trailer so the Python wire detects it purely by length; the
        // 28-byte header stays unchanged. j_len=0 until state_loop has a valid
        // reading -> legacy frame, and the consumer falls back to obs joints.
        const bool is_left = (fr->channel >= 5);
        std::array<float, NUM_JOINTS> jbuf;
        bool jvalid;
        {
            std::lock_guard<std::mutex> lk(g_joints.mu);
            jbuf   = is_left ? g_joints.left       : g_joints.right;
            jvalid = is_left ? g_joints.left_valid : g_joints.right_valid;
        }
        const size_t j_len = jvalid ? NUM_JOINTS * sizeof(float) : 0;

        constexpr size_t HEADER = 28;
        std::vector<uint8_t> buf(HEADER + d_len + f_len + cp_len + j_len);
        const uint32_t channel  = static_cast<uint32_t>(fr->channel);
        const uint32_t frame_id = static_cast<uint32_t>(fr->frame_id);
        const double   ts       = fr->ts;
        const uint32_t dlen32   = static_cast<uint32_t>(d_len);
        const uint32_t flen32   = static_cast<uint32_t>(f_len);
        const uint32_t cplen32  = static_cast<uint32_t>(cp_len);
        std::memcpy(buf.data() +  0, &channel,  4);
        std::memcpy(buf.data() +  4, &frame_id, 4);
        std::memcpy(buf.data() +  8, &ts,       8);
        std::memcpy(buf.data() + 16, &dlen32,   4);
        std::memcpy(buf.data() + 20, &flen32,   4);
        std::memcpy(buf.data() + 24, &cplen32,  4);

        uint8_t* body = buf.data() + HEADER;
        if (d_len)  { std::memcpy(body, d_blk->data(),  d_len);  body += d_len;  }
        if (f_len)  { std::memcpy(body, f_blk->data(),  f_len);  body += f_len;  }
        if (cp_len) { std::memcpy(body, cp_blk->data(), cp_len); body += cp_len; }
        if (j_len)  { std::memcpy(body, jbuf.data(),    j_len); }

        zmq::message_t msg(buf.data(), buf.size());
        std::lock_guard<std::mutex> lk(mu_);
        try {
            sock_.send(msg, zmq::send_flags::none);
        } catch (const zmq::error_t&) {
            // socket closing during shutdown — drop quietly.
        }
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t  sock_;
    std::mutex     mu_;
};

static std::unique_ptr<TactilePublisher> g_tactile_pub;

// ── hand bridge ───────────────────────────────────────────────────────────────

class HandBridge {
public:
    HandBridge(const std::string& side, sharpa::SharpaWave& hand, double state_hz)
        : side_(side), hand_(hand),
          state_dt_ms_(static_cast<int>(1000.0 / state_hz)),
          cmd_positions_(NUM_JOINTS, 0.0f)
    {}

    void start(const std::string& cmd_topic, const std::string& state_topic) {
        pub_ = std::make_unique<ChannelPublisher<HandState_>>(state_topic);
        pub_->InitChannel();

        sub_ = std::make_unique<ChannelSubscriber<HandCmd_>>(
            cmd_topic,
            [this](const void* msg) { on_cmd(msg); }
        );
        sub_->InitChannel();

        running_ = true;
        cmd_thread_   = std::thread(&HandBridge::cmd_loop,   this);
        state_thread_ = std::thread(&HandBridge::state_loop, this);
        std::cout << "[" << side_ << "] bridge started\n";
    }

    void stop() {
        running_ = false;
        if (cmd_thread_.joinable())   cmd_thread_.join();
        if (state_thread_.joinable()) state_thread_.join();
        sub_.reset();
        pub_.reset();
    }

private:
    void on_cmd(const void* raw) {
        const HandCmd_* msg = static_cast<const HandCmd_*>(raw);
        if (!msg || msg->motor_cmd().empty()) return;
        std::vector<float> pos(NUM_JOINTS, 0.0f);
        int n = std::min(static_cast<int>(msg->motor_cmd().size()), NUM_JOINTS);
        for (int i = 0; i < n; ++i) pos[i] = msg->motor_cmd()[i].q();
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_positions_ = std::move(pos);
    }

    void cmd_loop() {
        while (running_) {
            std::vector<float> pos;
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                pos = cmd_positions_;
            }
            auto err = hand_.set_joint_position(pos, false);
            if (err.code != 0)
                std::cerr << "[" << side_ << "] set_joint_position: " << err.message << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / CMD_HZ));
        }
    }

    void state_loop() {
        while (running_) {
            auto [err, angles_deg] = hand_.get_joint_position_degree();
            if (err.code == 0) {
                HandState_ msg;
                std::vector<MotorState_> states(NUM_JOINTS);
                int n = std::min(static_cast<int>(angles_deg.size()), NUM_JOINTS);
                for (int i = 0; i < n; ++i) states[i].q(angles_deg[i]);
                msg.motor_state(states);
                pub_->Write(msg);

                {
                    std::lock_guard<std::mutex> lk(g_joints.mu);
                    auto& dst   = (side_ == "left") ? g_joints.left  : g_joints.right;
                    bool& valid = (side_ == "left") ? g_joints.left_valid : g_joints.right_valid;
                    const int m = std::min(static_cast<int>(angles_deg.size()), NUM_JOINTS);
                    for (int i = 0; i < m; ++i) dst[i] = angles_deg[i];
                    valid = (m == NUM_JOINTS);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(state_dt_ms_));
        }
    }

    std::string side_;
    sharpa::SharpaWave& hand_;
    int state_dt_ms_;

    std::mutex cmd_mutex_;
    std::vector<float> cmd_positions_;
    std::atomic<bool> running_{false};

    std::unique_ptr<ChannelPublisher<HandState_>> pub_;
    std::unique_ptr<ChannelSubscriber<HandCmd_>> sub_;
    std::thread cmd_thread_;
    std::thread state_thread_;
};

// ── hand setup ────────────────────────────────────────────────────────────────

static sharpa::SharpaWave& connect_hand(const std::string& side, float speed,
                                         bool skip_tactile,
                                         int discovery_timeout_s = 300) {
    auto& manager  = sharpa::SharpaWaveManager::get_instance();
    auto  deadline = std::chrono::steady_clock::now()
                   + std::chrono::seconds(discovery_timeout_s);

    std::cout << "[" << side << "] waiting for device discovery (up to "
              << discovery_timeout_s << "s)...\n";

    while (std::chrono::steady_clock::now() < deadline) {
        if (!manager.get_all_device_sn().empty()) break;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto devices = manager.get_all_device_sn();
    if (devices.empty())
        throw std::runtime_error("[" + side + "] no devices found after "
                                 + std::to_string(discovery_timeout_s) + "s");

    std::cout << "[" << side << "] found " << devices.size() << " device(s):";
    for (const auto& d : devices) std::cout << " " << d;
    std::cout << "\n";

    sharpa::HandSide hs = (side == "left") ? sharpa::HandSide::LEFT
                                           : sharpa::HandSide::RIGHT;
    // We always let the SDK initialize tactile (skip_tactile=false here, regardless
    // of the user flag) so we have a control channel to the tactile board. With
    // skip_tactile=true the SDK reports "Tactile sensor not ready" when we try to
    // call set_parameter for JPEG disable.
    // We then either leave JPEG enabled (--tactile) or disable it via JSON below.
    auto& hand = manager.connect(hs, /*skip_tactile=*/false);

    auto check = [&](const std::string& name, sharpa::Error err) {
        if (err.code != 0)
            throw std::runtime_error("[" + side + "] " + name + ": " + err.message);
    };

    check("set_control_mode",   hand.set_control_mode(sharpa::ControlMode::POSITION));
    check("set_speed_coeff",    hand.set_speed_coeff(speed));
    check("set_current_coeff",  hand.set_current_coeff(0.6f));
    check("set_control_source", hand.set_control_source(sharpa::ControlSource::SDK));

    // Register tactile callback BEFORE hand.start() so we don't miss the first
    // frames (matches the SDK sample at SDK_500/sample/c++/sharpa_tactile_callback.cc:133).
    // Drops RAW; sends DEFORM + F6 + CONTACT_POINT over ZMQ to the workstation recorder.
    if (g_tactile_pub) {
        hand.set_tactile_callback([](sharpa::tactile::Frame::Ptr fr) {
            if (g_tactile_pub) g_tactile_pub->publish(fr);
        });
    }

    if (!hand.start())
        throw std::runtime_error("[" + side + "] start() returned false");

    if (skip_tactile) {
        // Disable tactile-JPEG transmission on the device — this is the ~30 Mb/s
        // stream that saturates a 100 Mb link when both hands are running.
        // SDK 5.0 form (see SDK_500/sample/c++/sharpa_tactile_fetch.cc:102).
        // Must run AFTER hand.start() — set_parameter rejects with
        // "Tactile sensor not ready" if the tactile subsystem hasn't started yet.
        const std::string kDisableJpeg =
            R"({"secret_function":"set_tactile_jpeg_enable","enable":false,"channel":-1})";
        auto err = hand.set_parameter(kDisableJpeg);
        if (err.code != 0)
            std::cerr << "[" << side << "] WARN: set_parameter(set_tactile_jpeg_enable=false): "
                      << err.message << "\n";
        else
            std::cout << "[" << side << "] tactile JPEG stream disabled\n";
        // Give the device a moment to actually stop transmitting before
        // attempting the next operation / connecting the second hand.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    hand.set_joint_position(std::vector<float>(NUM_JOINTS, 0.0f), true);
    std::cout << "[" << side << "] hand ready.\n";
    return hand;
}

static void disconnect_hand(sharpa::SharpaWave* hand, const std::string& side) {
    if (!hand) return;
    try {
        hand->set_joint_position(std::vector<float>(NUM_JOINTS, 0.0f), true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        hand->stop();
    } catch (const std::exception& e) {
        std::cerr << "[" << side << "] stop error: " << e.what() << "\n";
    }
    std::cout << "[" << side << "] disconnected.\n";
}

// ── signal handling ───────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running = false; }

// ── main ──────────────────────────────────────────────────────────────────────

static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " [--speed FLOAT] [--state-hz FLOAT] [--side left|right|both] [--tactile] [--tactile-port N]\n"
              << "  --speed FLOAT    hand speed coefficient (default: 0.5)\n"
              << "  --state-hz FLOAT state publish rate Hz (default: 50)\n"
              << "  --side STR       which hand(s) to bridge: left, right, or both (default: both)\n"
              << "  --tactile        keep device-side tactile JPEG stream on (default: off; ~30 Mb/s per hand)\n"
              << "  --tactile-port N publish DEFORM+F6+CONTACT_POINT over ZMQ PUB on this port (default: 7779; 0 to disable)\n";
}

int main(int argc, char* argv[]) {
    float speed    = 0.5f;
    float state_hz = 50.0f;
    std::string side = "both";
    bool enable_tactile = false;
    int  tactile_port = 7779;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
            speed = std::stof(argv[++i]);
        else if (!std::strcmp(argv[i], "--state-hz") && i + 1 < argc)
            state_hz = std::stof(argv[++i]);
        else if (!std::strcmp(argv[i], "--side") && i + 1 < argc)
            side = argv[++i];
        else if (!std::strcmp(argv[i], "--tactile"))
            enable_tactile = true;
        else if (!std::strcmp(argv[i], "--tactile-port") && i + 1 < argc)
            tactile_port = std::stoi(argv[++i]);
        else { print_usage(argv[0]); return 1; }
    }

    if (side != "left" && side != "right" && side != "both") {
        std::cerr << "ERROR: --side must be 'left', 'right', or 'both' (got '" << side << "')\n";
        return 1;
    }
    const bool want_left  = (side == "left"  || side == "both");
    const bool want_right = (side == "right" || side == "both");

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    ChannelFactory::Instance()->Init(0);

    // Bring up the tactile ZMQ publisher BEFORE connecting hands so that the
    // per-hand callback registration (inside connect_hand) sees a live socket.
    if (tactile_port > 0) {
        try {
            g_tactile_pub = std::make_unique<TactilePublisher>(tactile_port);
        } catch (const std::exception& e) {
            std::cerr << "[tactile] failed to bind PUB port " << tactile_port
                      << ": " << e.what() << " — continuing without tactile ZMQ\n";
        }
    } else {
        std::cout << "[tactile] ZMQ publisher disabled (--tactile-port 0)\n";
    }

    sharpa::SharpaWave* left_hand  = nullptr;
    sharpa::SharpaWave* right_hand = nullptr;

    try {
        std::cout << "Connecting to Sharpa hands (side=" << side
                  << ", tactile=" << (enable_tactile ? "on" : "off") << ")...\n";
        const bool skip_tactile = !enable_tactile;
        if (want_left)  left_hand  = &connect_hand("left",  speed, skip_tactile);
        if (want_right) right_hand = &connect_hand("right", speed, skip_tactile);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        sharpa::SharpaWaveManager::get_instance().disconnect_all();
        return 1;
    }

    std::unique_ptr<HandBridge> left_bridge, right_bridge;
    if (left_hand) {
        left_bridge = std::make_unique<HandBridge>("left", *left_hand, state_hz);
        left_bridge->start(TOPIC_LEFT_CMD, TOPIC_LEFT_STATE);
    }
    if (right_hand) {
        right_bridge = std::make_unique<HandBridge>("right", *right_hand, state_hz);
        right_bridge->start(TOPIC_RIGHT_CMD, TOPIC_RIGHT_STATE);
    }

    std::cout << "\nBridge running. Ctrl+C to stop.\n\n";
    while (g_running)
        std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "\nShutting down...\n";
    if (left_bridge)  left_bridge->stop();
    if (right_bridge) right_bridge->stop();

    disconnect_hand(left_hand,  "left");
    disconnect_hand(right_hand, "right");
    sharpa::SharpaWaveManager::get_instance().disconnect_all();

    // Intentionally NOT resetting g_tactile_pub here: the SDK does not document
    // whether hand.stop() drains in-flight tactile callbacks, so a callback
    // mid-publish() could still be holding the publisher's mutex when we
    // destroy it. The OS reclaims the socket and ZMQ context at process exit
    // — simpler and trivially race-free.

    return 0;
}
