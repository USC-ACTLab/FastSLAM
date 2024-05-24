#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "math-util.h"
#include <math.h>

TEST_CASE("Test Gaussian Sampling - negative variance") {
   REQUIRE( isnan( MathUtil::sampleNormal(1.0f, -1.0f) ) );
}

TEST_CASE("Test Gaussian Sampling - zero variance") {
   REQUIRE(MathUtil::sampleNormal(1.0f, 0.0f) == 1.0f);
}

TEST_CASE("Test world to body 2D - zero degree heading") {
   std::pair<float, float> world_frame_speed{1.0f, 1.0f};
   float heading_rad = 0.0f;
   auto target = std::make_pair(1.0f, 1.0f);

   auto res = MathUtil::worldToBody2D(world_frame_speed, heading_rad);

   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second,0.00001f) );
}

TEST_CASE("Test body to world 2D - zero degree heading") {
   std::pair<float, float> body_frame_speed{1.0f, 1.0f};
   float heading_rad = 0.0f;
   auto target = std::make_pair(1.0f, 1.0f);

   auto res = MathUtil::bodyToWorld2D(body_frame_speed, heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );
}

TEST_CASE("Test world to body 2D - 90 degree heading") {
   std::pair<float, float> world_frame_vel{1.0f, 0.0f};
   float heading_rad = M_PI / 2;
   auto target = std::make_pair(0.0f, -1.0f);

   auto res = MathUtil::worldToBody2D(world_frame_vel, heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );
}

TEST_CASE("Test body to world 2D - 90 degree heading") {
   std::pair<float, float> body_frame_vel{1.0f, 0.0f};
   float heading_rad = M_PI / 2;
   auto target = std::make_pair(0.0f, 1.0f);

   auto res = MathUtil::bodyToWorld2D(body_frame_vel, heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );
}

TEST_CASE("Test world to body 2D - 180 degree heading") {
   std::pair<float, float> world_frame_vel{1.0f, 0.0f};
   float heading_rad = M_PI;
   auto target = std::make_pair(-1.0f, 0.0f);

   auto res = MathUtil::worldToBody2D(world_frame_vel, heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );
}

TEST_CASE("Test body to world 2D - 180 degree heading") {
   std::pair<float, float> world_frame_vel{1.0f, 0.0f};
   float heading_rad = M_PI;
   auto target = std::make_pair(-1.0f, 0.0f);

   auto res = MathUtil::worldToBody2D(world_frame_vel, heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );
}

TEST_CASE("Test foward-inverse transform: worldToBody") {
   std::pair<float, float> world_frame_vel{1.0f, 0.0f};
   float heading_rad = M_PI / 4;
   auto target = world_frame_vel;

   auto res =
      MathUtil::bodyToWorld2D(MathUtil::worldToBody2D(world_frame_vel,
                                                      heading_rad), heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );
}

TEST_CASE("Test foward-inverse transform: bodyToWorld") {
   std::pair<float, float> world_frame_vel{1.0f, 0.0f};
   float heading_rad = M_PI / 4;
   auto target = world_frame_vel;

   auto res =
      MathUtil::worldToBody2D(MathUtil::bodyToWorld2D(world_frame_vel,
                                                      heading_rad), heading_rad);
   REQUIRE_THAT( res.first, Catch::Matchers::WithinRel(target.first, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.first, 0.00001f) );
   REQUIRE_THAT( res.second, Catch::Matchers::WithinRel(target.second, 0.001f) ||
                            Catch::Matchers::WithinAbs(target.second, 0.00001f) );

}
