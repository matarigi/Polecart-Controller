#include "PID_Functions.hpp"
#include "gtest/gtest.h"

TEST(PID_Functions, proportional_test_1) {
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(0, pid_func.proportional_calculator(0, 0));
}

TEST(PID_Functions, proportional_test_2) {
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(-10, pid_func.proportional_calculator(0, 10));
}

TEST(PID_Functions, proportional_test_3) {
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(10, pid_func.proportional_calculator(20, 10));
}

TEST(PID_Functions, proportional_error_4) {
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(0.8743647-9.23874676, pid_func.proportional_calculator(0.8743647, 9.23874676));
}

TEST(PID_Functions, proportional_error_5) {
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.proportional_calculator(-std::numeric_limits<double>::max(), 
        std::numeric_limits<double>::max()), std::overflow_error);
}

TEST(PID_Functions, proportional_error_6) {
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.proportional_calculator(std::numeric_limits<double>::max(), 
        -std::numeric_limits<double>::max()), std::overflow_error);
}