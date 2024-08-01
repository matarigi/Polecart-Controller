#include "PID_Functions.hpp"
#include "gtest/gtest.h"

// Test proportional function

TEST(PID_Proportional_Function, proportional_test_1) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(0, pid_func.proportional_calculator(0, 0));
}

TEST(PID_Proportional_Function, proportional_test_2) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(-10, pid_func.proportional_calculator(0, 10));
}

TEST(PID_Proportional_Function, proportional_test_3) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(10, pid_func.proportional_calculator(20, 10));
}

TEST(PID_Proportional_Function, proportional_test_4) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(0.8743647-9.23874676, pid_func.proportional_calculator(0.8743647, 9.23874676));
}

TEST(PID_Proportional_Function, proportional_test_5) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.proportional_calculator(-std::numeric_limits<double>::max(), 
        std::numeric_limits<double>::max()), std::overflow_error);
}

TEST(PID_Proportional_Function, proportional_test_6) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.proportional_calculator(std::numeric_limits<double>::max(), 
        -std::numeric_limits<double>::max()), std::overflow_error);
}

// Test integral function

TEST(PID_Integral_Function, integral_test_1) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(0,pid_func.integral_calculator(0,0,1));
}

TEST(PID_Integral_Function, integral_test_2) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(19,pid_func.integral_calculator(5,2,7));
}

TEST(PID_Integral_Function, integral_test_3) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(-157.34,pid_func.integral_calculator(5.45,-22.3,7.3));
}

TEST(PID_Integral_Function, integral_test_4) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.integral_calculator(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),7.3), std::overflow_error);
}

TEST(PID_Integral_Function, integral_test_5) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.integral_calculator(-std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),-std::numeric_limits<double>::max()), std::overflow_error);
}

// Test derivate function

TEST(PID_Derivate_Function, derivate_test_1) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(0, pid_func.derivate_calculator(0,0,1));
}

TEST(PID_Derivate_Function, derivate_test_2) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(-1, pid_func.derivate_calculator(5,7,2));
}

TEST(PID_Derivate_Function, derivate_test_3) 
{
    PID_Functions pid_func;

    EXPECT_FLOAT_EQ(5.07375, pid_func.derivate_calculator(5.1,-7.077,2.4));
}

TEST(PID_Derivate_Function, derivate_test_4) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.derivate_calculator(std::numeric_limits<double>::max(),
        -std::numeric_limits<double>::max(),2.4), std::overflow_error);
}

TEST(PID_Derivate_Function, derivate_test_5) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.derivate_calculator(-std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),2.4), std::overflow_error);
}

TEST(PID_Derivate_Function, derivate_test_6) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.derivate_calculator(std::numeric_limits<double>::max(),
        1,std::numeric_limits<double>::min()), std::overflow_error);
}

TEST(PID_Derivate_Function, derivate_test_7) 
{
    PID_Functions pid_func;

    ASSERT_THROW(pid_func.derivate_calculator(2,1,0), std::overflow_error);
}