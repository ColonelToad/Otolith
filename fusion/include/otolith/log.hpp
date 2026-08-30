#pragma once
// Binary log contract — mirrors sim/otolith_sim/logger.py (little-endian, x86).
// Header 32 B: magic "OTLG", version, row_bytes, reserved, dt, row_count
// Row    288 B: see logger.py docstring for field order.

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace otolith {

constexpr char kMagic[4] = {'O','T','L','G'};
constexpr uint32_t kVersion = 1;
constexpr uint32_t kRowBytes = 288;
constexpr size_t kHeaderSize = 32;

#pragma pack(push, 1)
struct LogHeader {
    char magic[4];
    uint32_t version;    // 1
    uint32_t row_bytes;  // 288
    uint32_t reserved;   // 0
    double dt;           // sim step (s)
    uint64_t row_count;  // patched on close; readers recompute from file size
};
static_assert(sizeof(LogHeader) == 32, "header size");

struct LogRow {
    double t;                       // 0
    double gyro[3];                 // 8
    double accel[3];                // 32
    double qj[12];                  // 56
    uint8_t contacts[4];            // 152  FL,FR,RL,RR
    uint8_t gt_contacts[4];         // 156
    double gt_pos[3];               // 160
    double gt_quat[4];              // 184  wxyz
    double gt_vel[3];               // 216
    double gt_rpy_rate[3];          // 240
    double gt_accel[3];             // 264  world
};
static_assert(sizeof(LogRow) == 288, "row size");
#pragma pack(pop)

// Throws std::runtime_error on bad magic/version/row_bytes or truncated file.
struct LogFile {
    LogHeader header{};
    std::vector<LogRow> rows;
};

LogFile read_log(const std::string& path);
void write_log(const std::string& path, double dt, const std::vector<LogRow>& rows);

} // namespace otolith
