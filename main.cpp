// Ackerman Steering Simulation Main file 

#include <iostream>

#include <fstream>
#include "dynamics_fun.cpp"

int main() {

    // Initialize state vector: [x, y, heading (rad), velocity]
    double state[4] = {0.0, 0.0, 0.0, 5.0};

    // Initialize control input vector
    double control[2] = {15.0 * (M_PI / 180.0), 0.0}; // [steering angle (deg), acceleration (m/s^2)]

    // [mass (kg), wheelbase L (m) for bicycle model, optional second length, unused]
    // L must be > 0 or yaw rate term divides by zero.
    double param[4] = {1500.0, 2.5, 2.5, 0.0}; // 

    // Set time step and total simulation time
    double dt = 0.01;
    double T = 20.0;

    // Open a file to write the state to
    std::ofstream file("state.txt");
    file << "time, x, y, heading, velocity" << std::endl;

    // Call dynamics function for each time step
    for (double t = 0.0; t < T; t += dt) {
        dynamics_fun(state, control, param, dt);
        file << t << ", " << state[0] << ", " << state[1] << ", " << state[2] << ", " << state[3] << std::endl;
    }

    // Close the file
    file.close();

    return 0;
}
