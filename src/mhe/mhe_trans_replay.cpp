/**
 * Offline replay of the TRANSLATIONAL-ONLY MHE on real flight data.
 * Computes a = R(q)·e3 from the measured quaternion and feeds [p,v] + [T,a].
 * Build: cmake --build . --target mhe_trans_replay
 */
#include "quadrotor_mpc/mhe/mhe_trans_controller.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <cmath>

using namespace quadrotor_mpc;

static Vec3 thrust_dir(double qw, double qx, double qy, double qz) {
    // third column of R(q) = R·e3
    return Vec3(2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1 - 2*(qx*qx + qy*qy));
}

int main(int argc, char** argv) {
    std::string path = (argc > 1) ? argv[1] : "../results/nmpc_mhe_v1.csv";
    std::ifstream f(path);
    if (!f.is_open()) { printf("cannot open %s\n", path.c_str()); return 1; }
    std::string line; std::getline(f, line);
    std::vector<std::vector<double>> rows;
    while (std::getline(f, line)) {
        std::vector<double> v; std::stringstream ss(line); std::string c;
        while (std::getline(ss, c, ',')) { try { v.push_back(std::stod(c)); } catch(...) { v.push_back(0); } }
        if (v.size() >= 18) rows.push_back(std::move(v));
    }
    printf("loaded %zu rows\n", rows.size());
    if (rows.empty()) return 1;

    double c_drag = (argc > 2) ? std::stod(argv[2]) : 0.0;
    MheTransController mhe;
    if (!mhe.init()) { printf("init failed\n"); return 1; }
    mhe.set_drag(c_drag);
    printf("drag c = %.4f\n", c_drag);
    MheTransEstimate prior;
    prior.pos = Vec3(rows[0][1], rows[0][2], rows[0][3]);
    prior.vel = Vec3(rows[0][4], rows[0][5], rows[0][6]);
    prior.m_hat = 0.90;   // WRONG (true ~1.05)
    prior.d_hat = Vec3::Zero();
    Eigen::Matrix<double,10,1> P;
    P.segment<3>(0).setConstant(0.05); P.segment<3>(3).setConstant(0.2);
    P(6) = 0.1; P.segment<3>(7).setConstant(0.1);  // tighter d prior
    mhe.reset(prior, P);

    int ok = 0, fail = 0; std::vector<double> mh;
    for (size_t k = 0; k < rows.size(); k += 3) {
        const auto& r = rows[k];
        Eigen::Matrix<double,6,1> y; y << r[1],r[2],r[3], r[4],r[5],r[6];
        double T = r[14];
        Vec3 a = thrust_dir(r[7], r[8], r[9], r[10]);
        mhe.push(y, T, a);
        int st = mhe.solve();
        mhe.propagate_prior(st == 0);
        (st == 0) ? ++ok : ++fail;
        auto e = mhe.get_estimate();
        mh.push_back(e.m_hat);
        if (k % 600 == 0)
            printf("  k=%4zu t=%.1f st=%d m_hat=%.4f |d|=%.2f\n", k, r[0], st, e.m_hat, e.d_hat.norm());
    }
    { std::ofstream of("/tmp/offline_m.csv"); of << "k,m_hat\n";
      for (size_t i = 0; i < mh.size(); ++i) of << i*3*0.01 << "," << mh[i] << "\n"; }
    double mean = 0, sd = 0; int h = mh.size()/2;
    for (size_t i = h; i < mh.size(); ++i) mean += mh[i]; mean /= (mh.size()-h);
    for (size_t i = h; i < mh.size(); ++i) sd += (mh[i]-mean)*(mh[i]-mean); sd = std::sqrt(sd/(mh.size()-h));
    printf("\n=== TRANS-MHE REPLAY ===\nsolver: %d ok, %d fail (%.0f%% ok)\n", ok, fail, 100.0*ok/(ok+fail));
    printf("m_hat: %.3f -> %.3f  mean(2nd half)=%.3f std=%.3f  (real ~1.05)\n", mh.front(), mh.back(), mean, sd);
    return 0;
}
