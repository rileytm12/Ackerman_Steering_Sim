// Control law: Δu = -K e, e = [x-xr, y-yr, wrap(ψ-ψr)]

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

using Eigen::VectorXd;

inline VectorXd wrapStateError_fun(double* state, double* traj)
{
    VectorXd err(3);
    err(0) = state[0] - traj[0];
    err(1) = state[1] - traj[1];
    err(2) = state[2] - traj[2];
    err(2) = std::atan2(std::sin(err(2)), std::cos(err(2)));
    return err;
}

inline VectorXd controlLaw_fun(
    double* state,
    double* traj,
    const std::vector<Eigen::MatrixXd>& K_seq,
    int k)
{
    if (K_seq.empty()) {
        return VectorXd::Zero(2);
    }
    const int kk = std::max(0, std::min(k, static_cast<int>(K_seq.size()) - 1));
    VectorXd state_err = wrapStateError_fun(state, traj);
    return -K_seq[static_cast<size_t>(kk)] * state_err;
}
