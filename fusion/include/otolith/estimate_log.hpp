#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace otolith {

constexpr char kEstMagic[4] = {'E','S','T','M'};
constexpr uint32_t kEstVersion = 1;
constexpr uint32_t kEstRowBytes = 136;
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
#pragma pack(pop)

struct EstFile {
    EstHeader header{};
    std::vector<EstRow> rows;
};

EstFile read_estimate(const std::string& path);
void write_estimate(const std::string& path, double dt, const std::vector<EstRow>& rows);

} // namespace otolith
