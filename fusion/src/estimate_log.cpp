#include "otolith/estimate_log.hpp"
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace otolith {

EstFile read_estimate(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("open failed: "+path);
    f.seekg(0,std::ios::end);
    size_t sz=(size_t)f.tellg();
    f.seekg(0);
    if (sz<kEstHeaderSize) throw std::runtime_error("too short");
    EstHeader h{};
    f.read((char*)&h,sizeof(h));
    if (!f) throw std::runtime_error("header read");
    if (std::memcmp(h.magic,kEstMagic,4)!=0) throw std::runtime_error("bad est magic");
    if (h.version!=kEstVersion && h.version!=kEstVersion1) throw std::runtime_error("est version");
    uint32_t expect = (h.version==kEstVersion) ? kEstRowBytes : kEstRowBytesV1;
    if (h.row_bytes!=expect) throw std::runtime_error("est row_bytes");
    size_t payload=sz-kEstHeaderSize;
    if (payload%h.row_bytes!=0) throw std::runtime_error("truncated est row");
    size_t n=payload/h.row_bytes;
    EstFile out; out.header=h; out.is_v2 = (h.version==kEstVersion);
    if (out.is_v2) {
        out.rows_v2.resize(n);
        if(n) f.read((char*)out.rows_v2.data(), n*sizeof(EstRowV2));
    } else {
        out.rows.resize(n);
        if(n) f.read((char*)out.rows.data(), n*sizeof(EstRow));
    }
    return out;
}
void write_estimate(const std::string& path, double dt, const std::vector<EstRow>& rows){
    EstHeader h{};
    std::memcpy(h.magic,kEstMagic,4);
    h.version=kEstVersion1; h.row_bytes=kEstRowBytesV1; h.reserved=0; h.dt=dt; h.row_count=rows.size();
    std::ofstream f(path, std::ios::binary|std::ios::trunc);
    if(!f) throw std::runtime_error("open write "+path);
    f.write((char*)&h,sizeof(h));
    if(!rows.empty()) f.write((char*)rows.data(), rows.size()*sizeof(EstRow));
}
void write_estimate_v2(const std::string& path, double dt, const std::vector<EstRowV2>& rows){
    EstHeader h{};
    std::memcpy(h.magic,kEstMagic,4);
    h.version=kEstVersion; h.row_bytes=kEstRowBytes; h.reserved=0; h.dt=dt; h.row_count=rows.size();
    std::ofstream f(path, std::ios::binary|std::ios::trunc);
    if(!f) throw std::runtime_error("open write "+path);
    f.write((char*)&h,sizeof(h));
    if(!rows.empty()) f.write((char*)rows.data(), rows.size()*sizeof(EstRowV2));
}

} // namespace otolith
