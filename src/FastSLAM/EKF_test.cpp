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
    struct Pose2D init_pose = { .x = 1, .y =0, .theta_rad = 0 };
    struct VelocityCommand2D init_cmd = { .vx_mps = 1, .wz_radps = 0 };
    struct Observation2D init_obs = { .range_m = 1, .bearing_rad = M_PI/2 };

    //this is 0, 1 WHICH IS CORRECT
    Point2D init_obs_cart = {
        init_obs.range_m * cosf(init_obs.bearing_rad),
        init_obs.range_m * sinf(init_obs.bearing_rad),
    };

    Eigen::Matrix2f init_cov;
    init_cov << 1, 0,
        0, 1;

    Eigen::Matrix3f process_noise;
    process_noise << 1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0;
    
#ifdef USE_MOCK
    std::shared_ptr<MockManager2D> test_manager =
        std::make_shared<MockManager2D>(init_pose, init_cmd, init_cov, 0, process_noise);
    std::shared_ptr<RobotManager2D> robot_instance(test_manager);
#elif defined(USE_SIM)
    std::shared_ptr<RobotManager2D> robot_instance =
        std::make_shared<Create3Manager>(init_pose, init_cmd, init_cov, 0, process_noise);
#endif

    std::unique_ptr<LMEKF2D> filled_landmark_ekf =
        std::make_unique<LMEKF2D>(init_obs_cart, init_cov, robot_instance);
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

    //TODO: needs numerical sim and mock manager
    SECTION("Test: one step updates correctly") {
#ifdef USE_MOCK
        //init pos, init obs, and init cmd have alr been given
        //lm in world view - correct
        struct Point2D world_obs = test_manager->inverseMeas(init_pose, init_obs);
        //init cmd for vel - give it the command
        test_manager->sampleControl();
        test_manager->sampleIMU();
        //returns new pose of robot after the control - correct
        struct Pose2D new_pose = test_manager->motionUpdate();

        // new obs - this is not working, and instead returns 0,0 ?
        test_manager->sampleLandMark();
        struct Observation2D new_obs = test_manager->getCurrObs();
        std::cout << "New observation: range =" << new_obs.range_m << ", bearing =" << new_obs.bearing_rad << std::endl;
        
        //update ekf
        filled_landmark_ekf->updateObservation(new_obs);

        struct Observation2D predicted = test_manager->predictMeas(world_obs);
        
        
        //i made measJacobian and calcKalman public in ekf.h for debugging
        //testing jacobian
        //auto res = filled_landmark_ekf->measJacobian();
        //std::cout << "jacobian" << res << std::endl;
        filled_landmark_ekf->update();

       
        //testing kalman
        //auto kal = filled_landmark_ekf->calcKalmanGain();
        //std::cout << "kalman" << kal<<  std::endl;

        //testing predict meas
        std::cout << "predict: " << predicted.range_m << ", " << predicted.bearing_rad << std::endl;
        
        
        Eigen::Vector2f expected_state(-0.1312, 1.3999);
        Eigen::Matrix2f expected_cov;
        expected_cov << 0.5000f, 0.f,
                        0.f, 0.6667f;
        struct Point2D actual_res = filled_landmark_ekf->getLMEst();
        std::cout << "estimate: " << actual_res.x << ", " << actual_res.y << std::endl;
        //Eigen::Matrix2f actual_cov = filled_landmark_ekf->getCovariance();

        REQUIRE_THAT(expected_state(0), Catch::Matchers::WithinRel(actual_res.x, 0.01f) ||
                                        Catch::Matchers::WithinAbs(actual_res.x, 0.01f));
        REQUIRE_THAT(expected_state(1), Catch::Matchers::WithinRel(actual_res.y, 0.01f) ||
                                        Catch::Matchers::WithinAbs(actual_res.y, 0.01f));
        //i initially had a getCovariance() and then got rid of it
        //REQUIRE((filled_landmark_ekf->getCovariance().isApprox(expected_cov, 0.01)));
    }
#endif //USE_MOCK

    SECTION("Test: calculation of correct observation correspondence") {
#ifdef USE_MOCK
        
        float cpd = filled_landmark_ekf->calcCPD();
        float expected_cpd = 0.0711f;
        REQUIRE_THAT(cpd, Catch::Matchers::WithinRel(expected_cpd, 0.01f));
#endif // USE_MOCK
    }
}
