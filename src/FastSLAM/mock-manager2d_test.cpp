#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "robot-manager.h"
#include <memory>
#include <cmath>

#ifdef USE_MOCK
TEST_CASE( "Test MockManager Numerical Sim" ) {
    // set-up
    struct Pose2D init_pose = { .x = 0, .y = 0, .theta_rad = 0 };
    struct VelocityCommand2D init_cmd = { .vx_mps = 1, .wz_radps = 0 };
    struct Point2D init_obs = { .x = 0, .y = 1 };
    Eigen::Matrix2f init_cov;
    init_cov << 1, 0,
        0, 1;
    std::unique_ptr<MockManager2D> test_manager = std::make_unique<MockManager2D>(init_pose, init_cmd, init_cov);

    SECTION( "MockManager: Test Motion Update" ){
        struct Pose2D expected = { .x = 1, .y = 0, .theta_rad = 0 };
        auto res = test_manager->motionUpdate();
        REQUIRE_THAT( res.x, Catch::Matchers::WithinRel(expected.x, 0.001f) ||
                      Catch::Matchers::WithinAbs(expected.x, 0.00001f) );
        REQUIRE_THAT( res.y, Catch::Matchers::WithinRel(expected.y, 0.001f) ||
                      Catch::Matchers::WithinAbs(expected.y, 0.00001f) );
        REQUIRE_THAT( res.theta_rad, Catch::Matchers::WithinRel(expected.theta_rad, 0.001f) ||
                      Catch::Matchers::WithinAbs(expected.theta_rad, 0.00001f) );
    }

    SECTION( "MockManager: test measurement prediction" ){
        // result obtained from numerical simulation
        // front x, right y, clockwise positive rotation

        auto curr_pose = test_manager->motionUpdate();

        SECTION( "Landmark Position 1: right rear of robot" ){
            auto res = test_manager->predictMeas({.x = 0.02, .y = 1.11});
            const struct Observation2D expected = { .range_m = 1.48071, .bearing_rad = 2.29407 };
            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) );
        }

        SECTION( "Landmark Position 2: right front of robot" ){
            auto res = test_manager->predictMeas({.x = 1.76, .y = 2.15});
            const struct Observation2D expected = { .range_m = 2.28037, .bearing_rad = 1.23102 };
            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) );
        }

        SECTION("Landmark Position 3: left rear of robot"){
            auto res = test_manager->predictMeas({.x = 0.78, .y = -1.37});
            const struct Observation2D expected = { .range_m = 1.38755, .bearing_rad = -1.73002 };
            test_manager->sampleIMU();
            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) );
        }

        SECTION("Landmark Position 4: left front of robot"){
            auto res = test_manager->predictMeas({.x = 1.2, .y = -0.7});
            const struct Observation2D expected = { .range_m = 0.7280, .bearing_rad = -1.2925 };
            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) );
        }

        SECTION("Landmark Position 5: Landmark directly behind robot"){
            auto res = test_manager->predictMeas({.x = -0.16, .y = 0});
            const struct Observation2D expected = { .range_m = 1.16, .bearing_rad = -M_PI };

            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) ||
                          Catch::Matchers::WithinRel(-expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(-expected.bearing_rad, 0.00001f));
        }

        SECTION( "Landmark Position 5: non-zero robot heading" ) {
            test_manager->setState({.x = 1, .y = 0, .theta_rad = 1.94});
            auto res = test_manager->predictMeas({.x = -0.16, .y = 0});
            const struct Observation2D expected = { .range_m = 1.16, .bearing_rad = 1.20159 };
            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) );
        }

        SECTION( "Landmark Position 5: non-zero robot heading, wrap-around" ) {
            test_manager->setState({.x = 1, .y = 0, .theta_rad = -1.94});
            auto res = test_manager->predictMeas({.x = -0.16, .y = 0});
            const struct Observation2D expected = { .range_m = 1.16, .bearing_rad = -1.20159 };
            REQUIRE_THAT( res.range_m, Catch::Matchers::WithinRel(expected.range_m, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.range_m, 0.00001f) );
            REQUIRE_THAT( res.bearing_rad, Catch::Matchers::WithinRel(expected.bearing_rad, 0.001f) ||
                          Catch::Matchers::WithinAbs(expected.bearing_rad, 0.00001f) );
        }
    }

    SECTION( "MockManager: test lidar measurement jacobian" ){

        SECTION( "Config 1: colliding robot + landmark" ){
            test_manager->setState({.x = 0, .y = 0, .theta_rad = 0});
            auto res = test_manager->measJacobian({.x = 0, .y = 0});
            //REQUIRE( res.hasNaN() );
        }

    }
}
#endif // USE_MOCK
