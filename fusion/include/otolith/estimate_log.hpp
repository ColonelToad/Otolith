#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace otolith {

constexpr char kEstMagic[4] = {'E','S','T','M'};
constexpr uint32_t kEstVersion = 2;          // v2 adds 225*8 covariance
constexpr uint32_t kEstVersion1 = 1;
constexpr uint32_t kEstRowBytes = 136 + 225*8; // 1936
constexpr uint32_t kEstRowBytesV1 = 136;
constexpr size_t kEstHeaderSize = 32;

#pragma pack(push, 1)
struct EstHeader {
    char magic[4];
    uint32_t version;
    uint32_t row_bytes;
    uint32_t reserved;
    double dt;
    uint64_t row_count;
};
static_assert(sizeof(EstHeader)==32);

struct EstRow {
    double t;
    double p[3];
    double quat[4]; // wxyz
    double v[3];
    double bg[3];
    double ba[3];
};
static_assert(sizeof(EstRow)==136);

struct EstRowV2 {
    EstRow base;
    double P[225]; // row-major 15x15
};
static_assert(sizeof(EstRowV2)==1936);
#pragma pack(pop)

struct EstFile {
    EstHeader header{};
    std::vector<EstRow> rows;      // for v1
    std::vector<EstRowV2> rows_v2; // for v2
    bool is_v2 = false;
};

EstFile read_estimate(const std::string& path);
void write_estimate(const std::string& path, double dt, const std::vector<EstRow>& rows);
void write_estimate_v2(const std::string& path, double dt, const std::vector<EstRowV2>& rows);

} // namespace otolith
