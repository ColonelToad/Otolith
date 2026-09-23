// Contender A bench sub: ROS 2 topics via zenoh RMW (ADR-0005).
// Spinner thread + stall monitor in main (2 s stall / 15 s grace, same
// policy as the SHM contenders). See common.hpp for CLI + protocol.

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include "common.hpp"

namespace {
std::vector<bench::Sample> g_samples;
std::mutex g_mu;
std::atomic<uint64_t> g_last_rx{0};
std::atomic<int> g_received{0};

void cb(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const uint64_t rx = bench::now_ns();
    if (msg->data.size() < 16) return;
    uint64_t tx = 0, seq = 0;
    std::memcpy(&tx, msg->data.data(), 8);
    std::memcpy(&seq, msg->data.data() + 8, 8);
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_samples.push_back({seq, rx - tx});
    }
    g_last_rx.store(rx, std::memory_order_relaxed);
    g_received.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace

int main(int argc, char** argv) {
    bench::Args a;
    if (!bench::parse_args(argc, argv, a)) return 2;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bench_a_sub");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto sub = node->create_subscription<std_msgs::msg::ByteMultiArray>(
        "/otolith/bench_a", qos, cb);
    if (!bench::touch(a.ready)) return 1;

    std::thread spinner([node]() { rclcpp::spin(node); });
    const uint64_t t_start = bench::now_ns();
    g_last_rx.store(t_start, std::memory_order_relaxed);
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const uint64_t now = bench::now_ns();
        const int n = g_received.load(std::memory_order_relaxed);
        const uint64_t last = g_last_rx.load(std::memory_order_relaxed);
        if (n == 0 && now - t_start > 15ull * 1000000000ull) break;
        if (n > 0 && now - last > 2ull * 1000000000ull) break;
    }
    rclcpp::shutdown();
    spinner.join();
    if (!bench::write_samples(a.out, g_samples)) return 1;
    return 0;
}
