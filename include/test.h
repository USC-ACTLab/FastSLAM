#pragma once
#include <iostream>
#include "Eigen/Core"

void test_func() {
    std::cout << "Test func" << std::endl;
}

void test_eigen() {
    Eigen::Vector2d test_vec(2.0, 2.0);
    std::cout << "Test eigen lib\n" << test_vec << std::endl;
}
