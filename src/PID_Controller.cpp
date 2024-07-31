#include "PID_Controller.hpp"

PID_Controller::PID_Controller() {
    this -> prev_error = 0;
    this -> integral = 0;
    this -> pid_func = PID_Functions();
    this -> constants = std::nullopt;
    this -> set_point = std::nullopt;
}

void PID_Controller::init(struct PID_Constants constants) {
    constants.k_derivate++;
    return;
}

void PID_Controller::set_set_point(double set_point) {
    set_point++;
    return;
}

double PID_Controller::output_pid_calculation(double input, double delta_time) {
    return input + delta_time;
}