// Discrete-time Jacobian linearization of kinematic bicycle (error on x, y, ψ only).

#include <cmath>
#include <Eigen/Dense>

void linearApprox_fun(double* state, double* control, double* param, double dt,
                      Eigen::Matrix3d& Ad, Eigen::Matrix<double, 3, 2>& Bd)
{
    const double v = state[3];
    const double theta = state[2];
    const double delta = control[0];
    const double L = param[1];

    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    const double sec2 = 1.0 / (std::cos(delta) * std::cos(delta));

    // Continuous-time A = ∂f/∂[x,y,θ]^T (f = [v cos θ, v sin θ, v/L tan δ]^T)
    Eigen::Matrix3d A;
    A << 0.0, 0.0, -v * sin_t,
        0.0, 0.0, v * cos_t,
        0.0, 0.0, 0.0;

    // B = ∂f/∂[δ, a]^T  (ẋ=v cos θ, ẏ=v sin θ, ψ̇=v/L tan δ, v̇=a — error state omits v, so couple a into ẋ,ẏ through v)
    Eigen::Matrix<double, 3, 2> B;
    B << 0.0, cos_t,
        0.0, sin_t,
        (v / L) * sec2, std::tan(delta) / L;

    Ad = Eigen::Matrix3d::Identity() + A * dt;
    Bd = B * dt;
}
