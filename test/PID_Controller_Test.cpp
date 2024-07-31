#include "PID_Controller.hpp"
#include "gtest/gtest.h"

TEST(PID_controller_init_test, null_struct_test) {
    PID_Controller controller;
    PID_Constants k;

    ASSERT_THROW(controller.init(k), std::invalid_argument);
}

TEST(PID_controller_output_test, null_set_point_test) {
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 0;
    k.k_integral = 0;
    k.k_proportional = 0;

    controller.init(k);

    ASSERT_THROW(controller.output_pid_calculation(0, 1), std::bad_optional_access);
}

TEST(PID_controller_output_test, null_constants_test) {
    PID_Controller controller;
    controller.set_set_point(0);

    ASSERT_THROW(controller.output_pid_calculation(0, 1), std::bad_optional_access);
}

TEST(PID_controller_output_test, divide_by_zero_test) {
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 0;
    k.k_integral = 0;
    k.k_proportional = 0;

    controller.init(k);

    controller.set_set_point(0);

    ASSERT_THROW(controller.output_pid_calculation(0, 0), std::invalid_argument);
}