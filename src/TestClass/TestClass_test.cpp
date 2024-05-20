#include <catch2/catch_test_macros.hpp>
#include "test.h"


TEST_CASE("test the first function") {
    REQUIRE(test_func() == 0);
}

TEST_CASE("test the eigen function") {
    Eigen::Vector2d correct_vec(2.0, 2.0);
    REQUIRE(test_eigen() == correct_vec);
}

TEST_CASE("test the test class") {
    TestClass a;
    TestClass* a_ptr = new TestClass();

    REQUIRE(a._public_test_int == 1);

    SECTION("test return_one function ") {
        REQUIRE(a.return_one() == 1);
    }

    SECTION("make call to function first, then check") {
        a.return_one();
        REQUIRE(a.return_one() == 1);
    }

    SECTION("make call using pointer"){
        REQUIRE(a_ptr->return_one() == 1);
    }

    // tear down
    delete a_ptr;
}
