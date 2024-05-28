#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "EKF.h"
#include "robot-manager.h"


TEST_CASE("Empty EKF"){
    // set up
    LMEKF2D* empty_landmark_ekf = new LMEKF2D();

    SECTION( "Test: prediction does not change state" ){
        struct Point2D expected_res = empty_landmark_ekf->getLMEst();
        empty_landmark_ekf->predict();
        struct Point2D actual_res = empty_landmark_ekf->getLMEst();

        REQUIRE_THAT( expected_res.x, Catch::Matchers::WithinRel(actual_res.x, 0.001f) ||
                      Catch::Matchers::WithinAbs(actual_res.x, 0.00001f) );
        REQUIRE_THAT( expected_res.y, Catch::Matchers::WithinRel(actual_res.y, 0.001f) ||
                      Catch::Matchers::WithinAbs(actual_res.y, 0.00001f) );
    }

    SECTION( "Test: empty robot manager handling" ) {

        SECTION("Test update function") {
            REQUIRE(empty_landmark_ekf->update() == KF_RET::EMPTY_ROBOT_MANAGER);
        }

        SECTION("Test calcCPD function") {
            KF_RET res = static_cast<KF_RET>( empty_landmark_ekf->calcCPD() );
            REQUIRE(res == KF_RET::EMPTY_ROBOT_MANAGER);
        }

    }

    // tear-down
    delete empty_landmark_ekf;
}

TEST_CASE("Non-empty EKF") {
    // set-up
    struct Pose2D init_pose = { .x = 0, .y =0, .theta_rad = 0 };
    struct VelocityCommand2D init_cmd = { .vx_mps = 0, .wz_radps = 0 };
    struct Point2D init_obs = { .x = 1, .y = 0 };
    Eigen::Matrix2f init_cov;
    init_cov << 1, 0,
        0, 1;
    std::shared_ptr<RobotManager2D> robot_instance = std::make_shared<Create3Manager>(init_pose, init_cmd, init_cov);

    LMEKF2D* filled_landmark_ekf = new LMEKF2D(init_obs, init_cov, robot_instance);

    SECTION("Test: prediction does not change state") {
        struct Point2D expected_res = filled_landmark_ekf->getLMEst();
        filled_landmark_ekf->predict();
        struct Point2D actual_res = filled_landmark_ekf->getLMEst();

        REQUIRE_THAT( expected_res.x, Catch::Matchers::WithinRel(actual_res.x, 0.001f) ||
                      Catch::Matchers::WithinAbs(actual_res.x, 0.00001f) );
        REQUIRE_THAT( expected_res.y, Catch::Matchers::WithinRel(actual_res.y, 0.001f) ||
                      Catch::Matchers::WithinAbs(actual_res.y, 0.00001f) );

    }

    SECTION("Test: one step updates correctly") {
        // TODO: requires mock-up robot manager
    }

    SECTION("Test: calculation of correct observation correspondence") {
        //TODO: requires mock-up robot manager
    }

    // tear-down
    delete filled_landmark_ekf;
    robot_instance.reset();
    REQUIRE(robot_instance.use_count() == 0); // check correct release of robot manager
}
