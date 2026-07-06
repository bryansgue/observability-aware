/**
 * Offline MHE replay on REAL recorded flight data — NO MuJoCo, NO ROS2.
 *
 * Feeds the measured odometry (p,v,q,ω) and the APPLIED control (T, ω_cmd) from
 * a real SiL CSV into the 18D MHE, step by step, and logs the mass estimate.
 *
 * This is the decisive test: it removes the real-time/QP-gremlin and the
 * synthetic-data circularity. If m̂ → real mass (~1.05) here, the MHE genuinely
 * estimates from real data. If it stays biased (~0.7-0.9), the bias is structural.
 *
 * CSV column order (save_csv_extended):
 *   0:t 1:px 2:py 3:pz 4:vx 5:vy 6:vz 7:qw 8:qx 9:qy 10:qz 11:wx 12:wy 13:wz
 *   14:T 15:wx_cmd 16:wy_cmd 17:wz_cmd ...
 *
 * Build: cmake --build . --target mhe_replay
 * Run:   ./mhe_replay ../results/param_est.csv
 */
#include "quadrotor_mpc/mhe/mhe_nmpc_controller.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cmath>

using namespace quadrotor_mpc;

int main(int argc, char** argv)
{
    std::string path = (argc > 1) ? argv[1] : "../results/param_est.csv";
    std::ifstream f(path);
    if (!f.is_open()) { printf("cannot open %s\n", path.c_str()); return 1; }

    std::string line;
    std::getline(f, line);  // header

    // Parse all rows into vectors of doubles
    std::vector<std::vector<double>> rows;
    while (std::getline(f, line)) {
        std::vector<double> v; v.reserve(60);
        std::stringstream ss(line); std::string cell;
        while (std::getline(ss, cell, ',')) {
            try { v.push_back(std::stod(cell)); } catch (...) { v.push_back(0.0); }
        }
        if (v.size() >= 18) rows.push_back(std::move(v));
    }
    printf("loaded %zu rows from %s\n", rows.size(), path.c_str());
    if (rows.empty()) return 1;

    // ── Init MHE with WRONG mass prior, d free ──────────────────────────
    MheNmpcController mhe;
    if (!mhe.init()) { printf("init failed\n"); return 1; }

    MheNmpcEstimate prior;
    prior.pos   = Vec3(rows[0][1], rows[0][2], rows[0][3]);
    prior.vel   = Vec3(rows[0][4], rows[0][5], rows[0][6]);
    prior.quat  = Quat4(rows[0][7], rows[0][8], rows[0][9], rows[0][10]);
    prior.omega = Vec3(rows[0][11], rows[0][12], rows[0][13]);
    prior.m_hat = 0.90;    // WRONG (true model ≈ 1.05)
    prior.tau_hat = 0.03;
    prior.d_hat = Vec3::Zero();

    Eigen::Matrix<double,18,1> P;
    P.segment<3>(0).setConstant(0.05);
    P.segment<3>(3).setConstant(0.2);
    P.segment<4>(6).setConstant(0.02);
    P.segment<3>(10).setConstant(0.2);
    P(13) = 0.1; P(14) = 0.02; P.segment<3>(15).setConstant(0.5);  // JOINT: estimate m, τ, d
    mhe.reset(prior, P);

    // ── Replay ──────────────────────────────────────────────────────────
    // Push every 3rd row → ~0.03 s spacing to match the MHE node dt (N=31, 1.0 s window)
    int n_ok = 0, n_fail = 0;
    std::vector<double> mhist;
    for (size_t k = 0; k < rows.size(); k += 3) {
        const auto& r = rows[k];
        Eigen::Matrix<double,13,1> y;
        y << r[1],r[2],r[3], r[4],r[5],r[6], r[7],r[8],r[9],r[10], r[11],r[12],r[13];
        Control4 u; u << r[14], r[15], r[16], r[17];   // [T, ω_cmd]
        mhe.push(y, u);
        int st = mhe.solve();
        mhe.propagate_prior(st == 0);
        if (st == 0) ++n_ok; else ++n_fail;
        MheNmpcEstimate e = mhe.get_estimate();
        mhist.push_back(e.m_hat);
        if (k % 700 == 0)
            printf("  k=%4zu t=%.1f  status=%d  m_hat=%.4f  tau=%.4f  |d|=%.2f\n",
                   k, r[0], st, e.m_hat, e.tau_hat, e.d_hat.norm());
    }
    // stats on second half (converged)
    double mean = 0, sd = 0; int h = mhist.size()/2;
    for (size_t i = h; i < mhist.size(); ++i) mean += mhist[i];
    mean /= (mhist.size() - h);
    for (size_t i = h; i < mhist.size(); ++i) sd += (mhist[i]-mean)*(mhist[i]-mean);
    sd = std::sqrt(sd / (mhist.size() - h));
    printf("\n=== REPLAY RESULT ===\n");
    printf("solver: %d ok, %d fail (%.0f%% ok)\n", n_ok, n_fail, 100.0*n_ok/rows.size());
    printf("m_hat: %.3f -> %.3f   mean(2nd half)=%.3f  std=%.3f   (real mass ~1.05)\n",
           mhist.front(), mhist.back(), mean, sd);
    return 0;
}
