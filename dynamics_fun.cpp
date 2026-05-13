// Ackerman Steering Simulation Dynamics Function

#include <iostream>
#include <cmath>

void dynamics_fun(double *state, double *control, double *param, double dt) {

    // Extract state and control inputs
    // state[2] is heading in RADIANS (integrated yaw angle)
    double x_prev = state[0];
    double y_prev = state[1];
    double heading_prev = state[2];
    double velocity_prev = state[3];

    // control[0]: steering angle in degrees (converted here to radians)
    double steering_angle = control[0];
    double acceleration = control[1];

    double mass = param[0];
    double length = param[1];      
    double wheelbase = param[2];

    // Ackerman / bicycle kinematics: yaw rate = (v / L) * tan(steering)
    double x_new = x_prev + velocity_prev * std::cos(heading_prev) * dt;
    double y_new = y_prev + velocity_prev * std::sin(heading_prev) * dt;
    double heading_new = heading_prev + (velocity_prev / length) * std::tan(steering_angle) * dt;
    double velocity_new = velocity_prev + acceleration * dt;

    // Unused in this kinematic model (kept for a future dynamic extension)
    (void)mass;
    (void)wheelbase;

    // Update the state
    state[0] = x_new;
    state[1] = y_new;
    state[2] = fmod(heading_new, 2 * M_PI);
    state[3] = velocity_new;

}