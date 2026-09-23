// Contender A bench pub: ROS 2 topics via zenoh RMW (ADR-0005).
// Best-effort QoS (sensor-streaming profile), 288 B ByteMultiArray.
// Waits for matched discovery before pacing so rendezvous cost is not
// booked as transport latency. See common.hpp for CLI + protocol.

#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include "common.hpp"

int main(int argc, char** argv) {
    bench::Args a;
    if (!bench::parse_args(argc, argv, a)) return 2;
    const uint64_t period_ns = uint64_t(1000000000.0 / a.rate_hz);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bench_a_pub");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto pub = node->create_publisher<std_msgs::msg::ByteMultiArray>("/otolith/bench_a", qos);

    // Discovery gate: don't start the clock until the peer is matched.
    const uint64_t t_disc0 = bench::now_ns();
    while (pub->get_subscription_count() == 0) {
        if (bench::now_ns() - t_disc0 > 10ull * 1000000000ull) {
            std::fprintf(stderr, "no subscriber matched in 10 s\n");
            rclcpp::shutdown();
            return 1;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(5));
    }

    int sent = 0;
    const uint64_t t0 = bench::now_ns();
    for (int i = 0; i < a.n; ++i) {
        std_msgs::msg::ByteMultiArray msg;
        msg.data.resize(otolith::kRowBytes, 0);
        const uint64_t tx = bench::now_ns();
        const uint64_t seq = uint64_t(i);
        std::memcpy(msg.data.data(), &tx, 8);
        std::memcpy(msg.data.data() + 8, &seq, 8);
        pub->publish(msg);
        ++sent;  // middleware-side drops (keep_last eviction) surface as
                 // transport drops via seq gaps, counted offline
        bench::sleep_until_ns(t0 + uint64_t(i + 1) * period_ns);
    }
    if (!bench::write_sidecar(a.out, sent, 0)) return 1;
    rclcpp::shutdown();
    return 0;
}
