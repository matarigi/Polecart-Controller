#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <optional>
#include "PID_Functions.hpp"

struct PID_Constants {
    double k_proportional;
    double k_derivate;
    double k_integral;
};

class PID_Controller
{
    public:
        PID_Controller();
        void init(struct PID_Constants PID_Constants);
        void set_set_point(double set_point);
        double output_pid_calculation(double input, double delta_time);
        double add_weight(double proportional, double derivate, double integral);

    private:
       std::optional<struct PID_Constants> constants;  
       std::optional<double> set_point;
       double prev_error;
       double integral;
       PID_Functions pid_func;
};

#endif