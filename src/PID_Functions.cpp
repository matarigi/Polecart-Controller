#include "PID_Functions.hpp"
#include <stdexcept>
#include <cmath>

double PID_Functions::proportional_calculator(double set_point, double measured_value) 
{
    double result = set_point - measured_value;

    if (std::isinf(result)) {
        throw std::overflow_error("Overflow error\n");
    } else {
        return result;
    }
}

double PID_Functions::derivate_calculator(double error, double prev_error, double delta_time) {
    return error + prev_error + delta_time;
}

double PID_Functions::integral_calculator(double prev_integral, double error, double delta_time) {
    return prev_integral + error + delta_time;
}