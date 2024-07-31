#include "PID_Functions.hpp"
#include <stdexcept>
#include <cmath>

double PID_Functions::proportional_calculator(double set_point, double measured_value) 
{
    double result = (double) set_point - (double) measured_value;

    return check_overflow(result);
}

double PID_Functions::derivate_calculator(double error, double prev_error, double delta_time) {

    if (delta_time == 0) {
        throw std::overflow_error("Overflow error\n");
    }

    double result = ((double) error - (double) prev_error) / (double) delta_time;

    return check_overflow(result);
}

double PID_Functions::integral_calculator(double prev_integral, double error, double delta_time) {

    double result = (double) prev_integral + ((double) error * (double) delta_time);

    return check_overflow(result);
}

/**
 * This method return the input if there is no overflow, otherwise throw std::overflow
 */
double PID_Functions::check_overflow(double result) {
    if (std::isinf(result)) {
        throw std::overflow_error("Overflow error\n");
    } else {
        return result;
    }
}