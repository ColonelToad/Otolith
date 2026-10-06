// Regenerates the C++ goldens asserted by
// `rust/otolith-fusion/src/fusion.rs::differential_vs_cpp_golden`.
//
// WHY THIS IS A CHECKED-IN TOOL
// =============================
// The Rust test's doc comment says its literals came from "`/tmp` golden.cpp".
// That file is gone, so the next time a leg constant moves -- and one just did,
// when L2 stopped being hypot(0.002, 0.213) and became 0.213 -- there was no
// way to regenerate them except by hand-editing 27 f64 literals. The Rust test
// then failed at a 1e-9 tolerance, correctly, with no indication of which
// numbers to trust.
//
// The goldens are a cross-language differential, so their whole value is that
// they are *verbatim C++ output at 17 digits*. That only works if there is a
// reliable way to re-emit them, so the generator lives in the repo now.
//
// USAGE
//   ./fusion/build/gen_golden          # prints Rust array literals to stdout
//
// Then paste the printed blocks into fusion.rs. Review the diff: a leg-geometry
// change should move q/v/p/bg/ba and the position diagonal, and nothing else.
// If `trace` alone moved, that is a numerical-conditioning artefact and needs a
// look rather than a paste.

#include "otolith/fusion.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>

using namespace otolith;

int main() {
    FusionEKF ekf;
    Eigen::Matrix<double, 12, 1> qj = Eigen::Matrix<double, 12, 1>::Zero();
    for (int leg = 0; leg < 4; ++leg) {
        qj[leg * 3 + 1] = 0.9;
        qj[leg * 3 + 2] = -1.8;
    }
    const Eigen::Vector3d gm(0.01, -0.01, 0.02);
    const Eigen::Vector3d am(0.05, -0.03, 9.82);
    int updates = 0;
    for (int i = 0; i < 200; ++i) {
        ekf.predict(0.002, gm, am);
        if (i % 5 == 0) {
            // update_legs takes the whole contact array and returns the number
            // of legs it updated, so it is called ONCE per step, not once per
            // leg. Calling it in a leg loop over-counts by 4x and also applies
            // four sequential updates per step.
            const std::array<uint8_t, 4> contact =
                (i % 10 == 5) ? std::array<uint8_t, 4>{0, 1, 0, 1}
                              : std::array<uint8_t, 4>{1, 0, 1, 0};
            updates += ekf.update_legs(qj, contact, gm, 0.002);
        }
    }

    const FusionState& s = ekf.state();
    // 17 significant digits, matching the Rust literals verbatim.
    std::printf("// updates = %d\n", updates);
    std::printf("let gq = [\n");
    std::printf("    %.17g,\n    %.17g,\n    %.17g,\n    %.17g,\n",
                s.q.x(), s.q.y(), s.q.z(), s.q.w());
    std::printf("];\n");
    auto vec3 = [](const char* name, const Eigen::Vector3d& v) {
        std::printf("// %s\nlet g_%s = [%.17g, %.17g, %.17g];\n",
                    name, name, v.x(), v.y(), v.z());
    };
    vec3("p", s.p);
    vec3("v", s.v);
    vec3("bg", s.bg);
    vec3("ba", s.ba);
    // p_cov is the full 15x15, so this is the trace of all of P.
    std::printf("// trace\n%.17g\n", s.P.trace());
    std::printf("// P diag\n");
    for (int i = 0; i < 15; ++i) std::printf("    %.17g,\n", s.P(i, i));
    return EXIT_SUCCESS;
}