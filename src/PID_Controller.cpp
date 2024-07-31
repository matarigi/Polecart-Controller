#include "PID_Controller.hpp"
#include <stdexcept>
#include <cmath>

PID_Controller::PID_Controller() {
    this -> prev_error = 0.0;
    this -> integral = 0.0;
    this -> pid_func = PID_Functions();
    this -> constants = std::nullopt;
    this -> set_point = std::nullopt;
}

void PID_Controller::init(struct PID_Constants constants) {
    this -> constants = std::optional<struct PID_Constants>(constants);

}

void PID_Controller::set_set_point(double set_point) {
    this -> set_point = std::optional<double>((double) set_point);
}

double PID_Controller::output_pid_calculation(double input, double delta_time) {
    double error = this -> pid_func.proportional_calculator(this -> set_point.value(), (double) input);
    double integral = this -> pid_func.integral_calculator((double) this -> integral, (double) error, (double) delta_time);
    double derivate = this -> pid_func.derivate_calculator((double) error, (double) this -> prev_error, (double) delta_time);

    double result = add_weight(error, derivate, integral);

    if (std::isinf(result)) {
        throw std::overflow_error("Overflow error\n");
    }

    this -> integral = integral;
    this -> prev_error = error;

    return result;
}

double PID_Controller::add_weight(double proportional, double derivate, double integral) {
    return ((double) this -> constants.value().k_proportional * proportional) +
        ((double) this -> constants.value().k_integral * integral) +
        ((double) this -> constants.value().k_derivate * derivate);
}