/**
 * @file test.h
 * @brief Test header for project and included libs
 */

#pragma once
#include <iostream>
#include "Eigen/Core"

/**
 * @brief test function that prints a msg
 * @return 0 for success
 */
int test_func() {
    std::cout << "Test func" << std::endl;
    return 0;
}

/**
 * @brief test function that invokes the eigen3 lib
 * @return a 2-d vector {2, 2}
 */
Eigen::Vector2d test_eigen() {
    Eigen::Vector2d test_vec(2.0, 2.0);
    std::cout << "Test eigen lib\n" << test_vec << std::endl;
    return test_vec;
}

class TestClass {
    public:
        /**
         * @brief test class constructor
         */
        TestClass() : _test_int(0), _public_test_int(1) {
            _test_mat << 1, 1, 1, 1;
        };

        /**
         * @brief Test function for unit testing
         * @return integer 1
         */
        int return_one() {return 1;};

        /**
         * @brief test class public integer member
         */
         int _public_test_int;

    private:

        /**
         * @brief test class private integer member
         */
        int _test_int;

        /**
         * @brief test class private vector member */
        Eigen::Matrix2d _test_mat;
};
