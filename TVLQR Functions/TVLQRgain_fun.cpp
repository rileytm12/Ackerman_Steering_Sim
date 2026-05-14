// TVLQR gain function (constant Ad, Bd over a finite horizon; apply K[0] each step).

#include <Eigen/Dense>
#include <vector>

struct TVLQRResult {
    std::vector<Eigen::MatrixXd> K;   // feedback gains K[k] ∈ R^{m×n}
    std::vector<Eigen::MatrixXd> P;   // cost-to-go P[k] ∈ R^{n×n}
};

inline TVLQRResult TVLQRgain_fun(
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& Qf,
    const Eigen::Matrix3d& Ad,
    const Eigen::Matrix<double, 3, 2>& Bd,
    int N_horizon)
{
    if (N_horizon < 1) {
        return {{}, {}};
    }

    Eigen::MatrixXd A = Ad;
    Eigen::MatrixXd B = Bd;

    std::vector<Eigen::MatrixXd> K_seq(static_cast<size_t>(N_horizon));
    std::vector<Eigen::MatrixXd> P_seq(static_cast<size_t>(N_horizon + 1));

    P_seq[static_cast<size_t>(N_horizon)] = Qf;

    for (int k = N_horizon - 1; k >= 0; --k) {
        const Eigen::MatrixXd& P_next = P_seq[static_cast<size_t>(k + 1)];

        Eigen::MatrixXd S = R + B.transpose() * P_next * B;
        Eigen::MatrixXd K = S.ldlt().solve(B.transpose() * P_next * A);
        K_seq[static_cast<size_t>(k)] = K;

        P_seq[static_cast<size_t>(k)] =
            Q + A.transpose() * P_next * A - A.transpose() * P_next * B * K;
    }

    return {K_seq, P_seq};
}
