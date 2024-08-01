#include "PID_Controller.hpp"
#include "gtest/gtest.h"

TEST(PID_controller_output_test, null_set_point_test) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 0;
    k.k_integral = 0;
    k.k_proportional = 0;

    controller.init(k);

    ASSERT_THROW(controller.output_pid_calculation(0, 1), std::bad_optional_access);
}

TEST(PID_controller_output_test, null_constants_test) 
{
    PID_Controller controller;
    controller.set_set_point(0);

    ASSERT_THROW(controller.output_pid_calculation(0, 1), std::bad_optional_access);
}

TEST(PID_controller_output_test, divide_by_zero_test) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 0;
    k.k_integral = 0;
    k.k_proportional = 0;

    controller.init(k);

    controller.set_set_point(0);

    ASSERT_THROW(controller.output_pid_calculation(0, 0), std::overflow_error);
}

TEST(PID_controller_output_test, overflow_test) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 0;
    k.k_integral = 0;
    k.k_proportional = std::numeric_limits<double>::max();

    controller.init(k);

    controller.set_set_point(std::numeric_limits<double>::max());

    ASSERT_THROW(controller.output_pid_calculation(0, 1), std::overflow_error);
}

TEST(PID_weight_test, weight_test1) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 1.2;
    k.k_integral = 1.3;
    k.k_proportional = -1.2;

    controller.init(k);

    EXPECT_FLOAT_EQ(-33.673,controller.add_weight(12.1, 32.8, -45.01));
}

TEST(PID_weight_test, weight_test2) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 87.3;
    k.k_integral = -15;
    k.k_proportional = 0;

    controller.init(k);

    EXPECT_FLOAT_EQ(3538.59,controller.add_weight(12.1, 32.8, -45.01));
}

TEST(PID_controller_output_test, good_test_1) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = -1.5;
    k.k_integral = -21;
    k.k_proportional = 13.71;

    controller.init(k);

    controller.set_set_point(5.97);

    EXPECT_FLOAT_EQ((10.4196-18.354-(114.0/115.0)), controller.output_pid_calculation(5.21, 1.15));
}

TEST(PID_controller_output_test, good_test_2) 
{
    PID_Controller controller;
    PID_Constants k;

    k.k_derivate = 15;
    k.k_integral = -0.09;
    k.k_proportional = 2;

    controller.init(k);

    controller.set_set_point(0.015);

    EXPECT_FLOAT_EQ((-3.97+0.20098125-(397.0/15.0)), controller.output_pid_calculation(2, 1.125));

    EXPECT_FLOAT_EQ((-1.97+0.3007125+(40.0/3.0)), controller.output_pid_calculation(1, 1.125));
}