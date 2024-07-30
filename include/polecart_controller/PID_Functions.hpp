#ifndef PID_FUNCTIONS_HPP
#define PID_FUNCTIONS_HPP

class PID_Functions
{
    public:
        double proportional_calculator(double set_point, double measured_value);
        double derivate_calculator(double error, double prev_error, double delta_time);
        double integral_calculator(double prev_integral, double error, double delta_time);
};

#endif