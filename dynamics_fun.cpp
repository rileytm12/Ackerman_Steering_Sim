// Kinematic bicycle: state [x, y, heading rad, v], inputs [steering rad, accel m/s^2]

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void dynamics_fun(double* state, double* control, double* param, double dt)
{
    const double x_prev = state[0];
    const double y_prev = state[1];
    const double heading_prev = state[2];
    const double velocity_prev = state[3];

    const double steering_angle = control[0];
    const double acceleration = control[1];

    const double mass = param[0];
    const double length = param[1];
    const double wheelbase = param[2];

    double x_new = x_prev + velocity_prev * std::cos(heading_prev) * dt;
    double y_new = y_prev + velocity_prev * std::sin(heading_prev) * dt;
    double heading_new = heading_prev;
    if (length > 0.0) {
        heading_new = heading_prev + (velocity_prev / length) * std::tan(steering_angle) * dt;
    }
    double velocity_new = velocity_prev + acceleration * dt;

    (void)mass;
    (void)wheelbase;

    state[0] = x_new;
    state[1] = y_new;
    state[2] = std::atan2(std::sin(heading_new), std::cos(heading_new));
    state[3] = velocity_new;
}
