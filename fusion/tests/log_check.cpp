#include "otolith/log.hpp"
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "usage: log_check <path>\n"; return 2; }
    try {
        auto lf = otolith::read_log(argv[1]);
        std::cout << lf.rows.size() << " " << lf.header.dt << "\n";
        if (!lf.rows.empty()) {
            // print first row t and gyro[0] as sanity
            std::cout << lf.rows[0].t << " " << lf.rows[0].gyro[0] << " "
                      << lf.rows.back().t << "\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
