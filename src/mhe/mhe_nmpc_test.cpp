/**
 * Standalone test for the 18D NMPC-MHE — NO MuJoCo, NO ROS2.
 *
 * Feeds SYNTHETIC, perfectly-consistent measurements (drone at hover with the
 * TRUE mass) and checks whether the solver returns status 0 and m̂ moves toward
 * the truth. Isolates "is the OCP/wrapper correct?" from the SiL rig + reset gremlin.
 *
 * Build:  cmake --build . --target mhe_nmpc_test
 * Run:    ./mhe_nmpc_test
 */
#include "quadrotor_mpc/mhe/mhe_nmpc_controller.hpp"
#include <cstdio>

using namespace quadrotor_mpc;

int main()
{
    const double g = 9.81;
    const double m_true = 1.08;
    const double m_init = 0.90;

    MheNmpcController mhe;
    if (!mhe.init()) { printf("init failed\n"); return 1; }

    MheNmpcEstimate prior;
    prior.pos = Vec3(0, 0, 1.5);
    prior.vel = Vec3::Zero();
    prior.quat = Quat4(1, 0, 0, 0);
    prior.omega = Vec3::Zero();
    prior.m_hat = m_init;
    prior.tau_hat = 0.03;
    prior.d_hat = Vec3::Zero();

    Eigen::Matrix<double,18,1> P;
    P.segment<3>(0).setConstant(0.05);
    P.segment<3>(3).setConstant(0.2);
    P.segment<4>(6).setConstant(0.02);
    P.segment<3>(10).setConstant(0.2);
    P(13) = 0.1; P(14) = 0.01; P.segment<3>(15).setConstant(0.5);
    mhe.reset(prior, P);

    // Synthetic measurement: drone hovering at rest at (0,0,1.5), level.
    // Applied control = TRUE hover thrust m_true*g, zero rates.
    Eigen::Matrix<double,13,1> y;
    y << 0, 0, 1.5,  0, 0, 0,  1, 0, 0, 0,  0, 0, 0;
    Control4 u;
    u << m_true * g, 0, 0, 0;   // [T, ω_cmd]

    printf("step | status | m_hat   | sigma\n");
    for (int k = 0; k < 60; ++k) {
        mhe.push(y, u);
        int st = mhe.solve();
        mhe.propagate_prior(st == 0);
        MheNmpcEstimate e = mhe.get_estimate();
        if (k % 5 == 0 || st != 0)
            printf("%4d |   %d    | %.4f | %.4e\n", k, st, e.m_hat, mhe.get_sigma());
    }
    return 0;
}
