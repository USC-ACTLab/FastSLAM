/**
 * @file test.h
 * @brief Test header for project and included libs
 */

#pragma once
#include <iostream>
#include "Eigen/Core"

/**
 * @brief test function that prints a msg
 */
void test_func() {
    std::cout << "Test func" << std::endl;
}

/**
 * @brief test function that invokes the eigen3 lib
 */
void test_eigen() {
    Eigen::Vector2d test_vec(2.0, 2.0);
    std::cout << "Test eigen lib\n" << test_vec << std::endl;
}

class TestClass {
    public:
        /**
         * @brief test class constructor
         */
        TestClass();
        virtual ~TestClass();
    private:

        /**
         * @brief test class private integer member
         */
        int test_int;


        /**
         * @brief test class private vector member */
        Eigen::Matrix2d test_mat;
};
