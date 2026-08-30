#include "otolith/log.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <filesystem>
#include <vector>

using namespace otolith;

TEST_CASE("LogRow layout matches Python contract", "[log]") {
    REQUIRE(sizeof(LogHeader) == 32);
    REQUIRE(sizeof(LogRow) == 288);
    REQUIRE(offsetof(LogRow, t) == 0);
    REQUIRE(offsetof(LogRow, gyro) == 8);
    REQUIRE(offsetof(LogRow, accel) == 32);
    REQUIRE(offsetof(LogRow, qj) == 56);
    REQUIRE(offsetof(LogRow, contacts) == 152);
    REQUIRE(offsetof(LogRow, gt_contacts) == 156);
    REQUIRE(offsetof(LogRow, gt_pos) == 160);
    REQUIRE(offsetof(LogRow, gt_quat) == 184);
    REQUIRE(offsetof(LogRow, gt_vel) == 216);
    REQUIRE(offsetof(LogRow, gt_rpy_rate) == 240);
    REQUIRE(offsetof(LogRow, gt_accel) == 264);
}

TEST_CASE("C++ round-trip write/read", "[log]") {
    namespace fs = std::filesystem;
    auto tmp = fs::temp_directory_path() / "otolith_log_roundtrip.bin";

    std::vector<LogRow> rows(3);
    for (int i = 0; i < 3; ++i) {
        LogRow r{};
        r.t = i * 0.002;
        for (int k = 0; k < 3; ++k) { r.gyro[k] = 0.1*i + 0.01*k; r.accel[k] = 9.81 + 0.02*i; }
        for (int k = 0; k < 12; ++k) r.qj[k] = 0.1*k + 0.01*i;
        r.contacts[0]=1; r.contacts[1]=0; r.contacts[2]=1; r.contacts[3]=0;
        r.gt_contacts[0]=1; r.gt_contacts[1]=0; r.gt_contacts[2]=1; r.gt_contacts[3]=0;
        r.gt_pos[0]=0.2*i; r.gt_pos[1]=0; r.gt_pos[2]=0.27;
        r.gt_quat[0]=1; r.gt_quat[1]=0; r.gt_quat[2]=0; r.gt_quat[3]=0;
        r.gt_vel[0]=0.2; r.gt_vel[1]=0; r.gt_vel[2]=0;
        r.gt_rpy_rate[0]=0.01*i; r.gt_rpy_rate[1]=0; r.gt_rpy_rate[2]=0;
        r.gt_accel[0]=0; r.gt_accel[1]=0; r.gt_accel[2]=0;
        rows[i]=r;
    }

    write_log(tmp.string(), 0.002, rows);
    auto lf = read_log(tmp.string());
    REQUIRE(lf.header.dt == Catch::Approx(0.002));
    REQUIRE(lf.rows.size() == 3);
    for (int i = 0; i < 3; ++i) {
        CHECK(lf.rows[i].t == Catch::Approx(rows[i].t));
        for (int k=0;k<3;++k) CHECK(lf.rows[i].gyro[k] == Catch::Approx(rows[i].gyro[k]));
        for (int k=0;k<12;++k) CHECK(lf.rows[i].qj[k] == Catch::Approx(rows[i].qj[k]));
        for (int k=0;k<4;++k) CHECK(lf.rows[i].contacts[k] == rows[i].contacts[k]);
        for (int k=0;k<3;++k) CHECK(lf.rows[i].gt_pos[k] == Catch::Approx(rows[i].gt_pos[k]));
    }
    fs::remove(tmp);
}

TEST_CASE("empty log", "[log]") {
    namespace fs = std::filesystem;
    auto tmp = fs::temp_directory_path() / "otolith_log_empty.bin";
    write_log(tmp.string(), 0.002, {});
    auto lf = read_log(tmp.string());
    REQUIRE(lf.rows.empty());
    REQUIRE(lf.header.dt == Catch::Approx(0.002));
    fs::remove(tmp);
}
