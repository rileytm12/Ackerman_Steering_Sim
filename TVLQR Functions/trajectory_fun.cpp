// Non–self-intersecting ribbon: x = vx*t (vx>0) ⇒ no self-cross in the x–y plot.
// Tuned milder amplitudes / longer periods so curvature stays easy for TV-LQR + steer limits.

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr double kEps = 1e-9;

inline void pack_traj(
    double x, double y, double xd, double yd, double xdd, double ydd, double L, double v_cmd, double traj[5])
{
    const double spd2 = xd * xd + yd * yd;
    double heading = 0.0;
    double heading_dot = 0.0;
    if (spd2 > kEps * kEps) {
        heading = std::atan2(yd, xd);
        heading_dot = (xd * ydd - yd * xdd) / spd2;
    }
    double steer = 0.0;
    if (std::abs(v_cmd) > kEps) {
        steer = std::atan(L * heading_dot / v_cmd);
    }
    traj[0] = x;
    traj[1] = y;
    traj[2] = heading;
    traj[3] = v_cmd;
    traj[4] = steer;
}

} // namespace

void trajectory_fun(double t, const double* x0, const double* param, double traj[5])
{
    const double L = param[1];
    if (L <= 0.0) {
        traj[0] = traj[1] = traj[2] = traj[3] = traj[4] = 0.0;
        return;
    }

    const double v_cmd = (x0 != nullptr && x0[3] > 0.3) ? x0[3] : 5.0;

    // Along-track speed for reference geometry (matches imposed ẋ for analytic feedforward)
    constexpr double vx = 4.35;

    // Softer weave: smaller A and slower ω ⇒ lower κ and steer rate (easier to track).
    constexpr double A1 = 3.0;
    constexpr double A2 = 1.85;
    constexpr double A3 = 1.05;
    constexpr double w1 = 2.0 * M_PI / 7.8;
    constexpr double w2 = 2.0 * M_PI / 2.65;
    constexpr double w3 = 2.0 * M_PI / 4.6;

    constexpr double k_sidewalk = 0.045;

    constexpr double A_sweep = 5.5;
    constexpr double T_sweep = 58.0; // s — one gentle bulge over a full run

    const double x = vx * t;
    const double y = A1 * std::sin(w1 * t) + A2 * std::sin(w2 * t) + A3 * std::sin(w3 * t) + k_sidewalk * t
        + A_sweep * std::sin(2.0 * M_PI * t / T_sweep);

    const double xd = vx;
    const double yd = A1 * w1 * std::cos(w1 * t) + A2 * w2 * std::cos(w2 * t) + A3 * w3 * std::cos(w3 * t)
        + k_sidewalk + A_sweep * (2.0 * M_PI / T_sweep) * std::cos(2.0 * M_PI * t / T_sweep);

    const double ydd = -A1 * w1 * w1 * std::sin(w1 * t) - A2 * w2 * w2 * std::sin(w2 * t)
        - A3 * w3 * w3 * std::sin(w3 * t)
        - A_sweep * (2.0 * M_PI / T_sweep) * (2.0 * M_PI / T_sweep) * std::sin(2.0 * M_PI * t / T_sweep);

    const double xdd = 0.0;
    pack_traj(x, y, xd, yd, xdd, ydd, L, v_cmd, traj);
}
