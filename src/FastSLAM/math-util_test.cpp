#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "math-util.h"
#include <math.h>

TEST_CASE("Test overloaded 2D point addition operator") {
   struct Point2D init = {.x = 1.2f, .y = -0.5f};
   Eigen::Vector2f corrector(0.5f, 1.5f);

   struct Point2D res = { .x = 1.7f, .y = 1.0f };
   init += corrector;
   REQUIRE_THAT( init.x, Catch::Matchers::WithinRel(res.x, 0.001f) ||
                            Catch::Matchers::WithinAbs(res.x, 0.00001f) );
   REQUIRE_THAT( init.y, Catch::Matchers::WithinRel(res.y, 0.001f) ||
                            Catch::Matchers::WithinAbs(res.y, 0.00001f) );

}

TEST_CASE("Test overloaded observation diff operator") {

   struct Observation2D one = { .range_m = 0, .bearing_rad = M_PI/2 };
   struct Observation2D two = { .range_m = 1, .bearing_rad = 0 };
   Eigen::Vector2f res(-1, M_PI/2);

   REQUIRE(one - two == res);
   REQUIRE(two - one == -res);

}

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

TEST_CASE( "Test CDF table generation" ){

   SECTION( "test vector of float" ){
      std::vector<float> pdf {0.1, 0.3, 0.4, 0.2};
      std::vector<float> cdf_target {0.1, 0.4, 0.8, 1};
      std::vector<float> cdf;
      MathUtil::genCDF(pdf, cdf);
      int idx = 0;
      for (const auto& it: cdf_target) {
         REQUIRE_THAT( it, Catch::Matchers::WithinRel(cdf[idx], 0.001f) ||
                       Catch::Matchers::WithinAbs(cdf[idx], 0.00001f) );
         idx++;
      }
   }

   SECTION( "test vector of int" ){
      std::vector<int> pdf {0, 1, 2, 3};
      std::vector<int> cdf_target {0, 1, 3, 6};
      std::vector<int> cdf;
      MathUtil::genCDF(pdf, cdf);
      int idx = 0;
      for (const auto& it: cdf_target) {
         REQUIRE( it == cdf[idx] );
         idx++;
      }
   }

   SECTION( "test empty vector" ){
      std::vector<int> pdf;
      std::vector<int> cdf;
      MathUtil::genCDF(pdf, cdf);
      REQUIRE( cdf.empty() );
   }
}
