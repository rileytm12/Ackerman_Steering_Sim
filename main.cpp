// Ackerman / bicycle sim — TV-LQR on a smooth forward-only weave (no path self-cross).

#include <algorithm>
#include <cmath>
#include <fstream>

#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "dynamics_fun.cpp"
#include "TVLQR Functions/trajectory_fun.cpp"
#include "TVLQR Functions/linearApprox_fun.cpp"
#include "TVLQR Functions/TVLQRgain_fun.cpp"
#include "TVLQR Functions/controlLaw_fun.cpp"

int main()
{
    const double v0 = 5.0;
    double x0[4] = {0.0, 0.0, 0.0, v0};
    double state[4] = {0.0, 0.0, 0.0, v0};

    double control[2] = {0.0, 0.0};

    double param[4] = {1500.0, 2.5, 2.5, 0.0};

    const double dt = 0.01;
    const double T = 58.0;
    const int N_lqr = 90;

    // Milder reference: still tight tracking, slightly less steering penalty so corrections stay smooth.
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
    Q(0, 0) = 85.0;
    Q(1, 1) = 85.0;
    Q(2, 2) = 42.0;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
    R(0, 0) = 2.6;
    R(1, 1) = 0.11;
    Eigen::MatrixXd Qf = Q * 3.0;

    std::ofstream state_file("state.txt");
    state_file << "time, x, y, heading, velocity" << std::endl;

    std::ofstream traj_file("trajectory.txt");
    traj_file << "time, x_ref, y_ref, heading_ref, v_ref, steering_ref" << std::endl;

    double traj[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    trajectory_fun(0.0, x0, param, traj);
    state[0] = traj[0];
    state[1] = traj[1];
    state[2] = traj[2];
    state[3] = v0;
    control[0] = traj[4];
    control[1] = 0.0;

    Eigen::Matrix3d Ad;
    Eigen::Matrix<double, 3, 2> Bd;

    const double steer_max = 52.0 * (M_PI / 180.0);
    const double accel_max = 5.0;

    for (double t = 0.0; t < T; t += dt) {
        trajectory_fun(t, x0, param, traj);

        linearApprox_fun(state, control, param, dt, Ad, Bd);

        TVLQRResult result_TVLQR = TVLQRgain_fun(Q, R, Qf, Ad, Bd, N_lqr);

        Eigen::VectorXd u = controlLaw_fun(state, traj, result_TVLQR.K, 0);

        // Feedforward nominal + LQR correction
        control[0] = traj[4] + u(0);
        control[1] = u(1) + 2.4 * (traj[3] - state[3]);
        control[0] = std::max(-steer_max, std::min(steer_max, control[0]));
        control[1] = std::max(-accel_max, std::min(accel_max, control[1]));

        dynamics_fun(state, control, param, dt);

        state_file << t << ", " << state[0] << ", " << state[1] << ", " << state[2] << ", " << state[3]
                   << std::endl;

        traj_file << t << ", " << traj[0] << ", " << traj[1] << ", " << traj[2] << ", " << traj[3] << ", "
                  << traj[4] << std::endl;
    }

    state_file.close();
    traj_file.close();
    return 0;
}
