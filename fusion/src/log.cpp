#include "otolith/log.hpp"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace otolith {

LogFile read_log(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("open failed: " + path);
    f.seekg(0, std::ios::end);
    auto sz = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    if (sz < kHeaderSize) throw std::runtime_error("file too short");

    LogHeader hdr{};
    f.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!f) throw std::runtime_error("header read failed");
    if (std::memcmp(hdr.magic, kMagic, 4) != 0) throw std::runtime_error("bad magic");
    if (hdr.version != kVersion) throw std::runtime_error("unsupported version");
    if (hdr.row_bytes != kRowBytes) throw std::runtime_error("row_bytes mismatch");

    size_t payload = sz - kHeaderSize;
    if (payload % kRowBytes != 0) throw std::runtime_error("truncated row");
    size_t n = payload / kRowBytes;

    LogFile out;
    out.header = hdr;
    out.rows.resize(n);
    if (n > 0) {
        f.read(reinterpret_cast<char*>(out.rows.data()), n * sizeof(LogRow));
        if (!f) throw std::runtime_error("row read failed");
    }
    return out;
}

void write_log(const std::string& path, double dt, const std::vector<LogRow>& rows) {
    LogHeader hdr{};
    std::memcpy(hdr.magic, kMagic, 4);
    hdr.version = kVersion;
    hdr.row_bytes = kRowBytes;
    hdr.reserved = 0;
    hdr.dt = dt;
    hdr.row_count = rows.size();

    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) throw std::runtime_error("open for write failed: " + path);
    f.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (!rows.empty()) {
        f.write(reinterpret_cast<const char*>(rows.data()), rows.size() * sizeof(LogRow));
    }
    if (!f) throw std::runtime_error("write failed");
}

} // namespace otolith
