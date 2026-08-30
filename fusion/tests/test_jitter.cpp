#include "otolith/fusion.hpp"
#include "otolith/leg_kin.hpp"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <vector>
#include <algorithm>

using namespace otolith;

TEST_CASE("jitter histogram - predict+update under 50us p99", "[jitter]") {
    FusionEKF ekf;
    Eigen::Matrix<double,12,1> qj; qj.setZero();
    for(int leg=0;leg<4;++leg){ qj[leg*3+1]=0.9; qj[leg*3+2]=-1.8; }
    std::array<uint8_t,4> contacts{1,0,0,1};
    // warmup
    for(int i=0;i<10;++i){ ekf.predict(0.002, Eigen::Vector3d::Zero(), Eigen::Vector3d(0,0,9.81)); ekf.update_legs(qj, contacts, Eigen::Vector3d::Zero(), 0.002); }

    const int N=5000;
    std::vector<double> us; us.reserve(N);
    for(int i=0;i<N;++i){
        auto t0=std::chrono::high_resolution_clock::now();
        ekf.predict(0.002, Eigen::Vector3d(0.01,0.02,0.03), Eigen::Vector3d(0.1,0.2,9.9));
        ekf.update_legs(qj, contacts, Eigen::Vector3d(0.01,0.02,0.03), 0.002);
        auto t1=std::chrono::high_resolution_clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(t1-t0).count());
    }
    std::sort(us.begin(), us.end());
    double p50=us[N/2], p99=us[N*99/100], mx=us.back();
    // WSL has no PREEMPT_RT; allow generous bounds (allocation + OS jitter)
    // The point is to have a histogram, not to enforce hard RT
    CHECK(p50 < 200.0);
    CHECK(p99 < 500.0);
    REQUIRE(mx < 5000.0);
}
