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
    struct VelocityCommand2D init_cmd = { .vx_mps = 1, .wz_radps = 0 };
    struct Observation2D init_obs = { .range_m = 1, .bearing_rad = M_PI/2 };
    struct Point2D init_belief = {.x = 0.0f, .y = 1.0f};
    Eigen::Matrix2f init_cov = Eigen::Matrix2f::Identity();

    Eigen::Matrix3f process_noise = Eigen::Matrix3f::Identity();

#ifdef USE_MOCK
    std::shared_ptr<MockManager2D> test_manager =
        std::make_shared<MockManager2D>(init_pose, init_cmd, init_cov, 0, process_noise);
    std::shared_ptr<RobotManager2D> robot_instance(test_manager);
#elif defined(USE_SIM)
    std::shared_ptr<RobotManager2D> robot_instance =
        std::make_shared<Create3Manager>(init_pose, init_cmd, init_cov, 0, process_noise);
#endif

    std::unique_ptr<LMEKF2D> filled_landmark_ekf =
        std::make_unique<LMEKF2D>(init_belief, init_cov, robot_instance);

#ifdef USE_MOCK
    REQUIRE(robot_instance.use_count() == 3); // check correct ownership changes
#endif //USE_MOCK

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
#ifdef USE_MOCK
        struct Pose2D new_pose = test_manager->motionUpdate();
        struct Observation2D new_obs = {.range_m = 1.5f, .bearing_rad = M_PI_4f};
        test_manager->setObs(new_obs);

        //update ekf
        filled_landmark_ekf->updateObservation(test_manager->getCurrObs());

        // check CPD before updating EKF
        float cpd = filled_landmark_ekf->calcCPD();
        float expected_cpd = 0.0452f;
        REQUIRE_THAT(cpd, Catch::Matchers::WithinRel(expected_cpd, 0.01f));

        filled_landmark_ekf->update();

        Eigen::Vector2f expected_state(-0.5857f, 1.4950f);
        Eigen::Matrix2f expected_cov;
        expected_cov << 0.5000f, 0.f,
                        0.f, 0.6667f;
        struct Point2D actual_res = filled_landmark_ekf->getLMEst();

        REQUIRE_THAT(expected_state(0), Catch::Matchers::WithinRel(actual_res.x, 0.01f) ||
                                        Catch::Matchers::WithinAbs(actual_res.x, 0.01f));
        REQUIRE_THAT(expected_state(1), Catch::Matchers::WithinRel(actual_res.y, 0.01f) ||
                                        Catch::Matchers::WithinAbs(actual_res.y, 0.01f));
#endif //USE_MOCK
    }

}
