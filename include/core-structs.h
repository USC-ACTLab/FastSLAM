/**
 * @file core-structs.h
 * @brief Defines the basic structs used to implement FastSLAM 1.0.
 */

#pragma once

#include <optional>
#include <Eigen/Dense>

/** A point (position only) in the 2D plane. */
struct Point2D {
   float x;           // x-coordinate (scaled in meters)
   float y;           // y-coordinate (scaled in meters)

   struct Point2D& operator+=(const Eigen::Vector2f& rhs) {
      x += rhs[0];
      y += rhs[1];
      return *this;
   }
};

/** A pose (position and orientation) in the 2D plane. */
struct Pose2D {
   float x;           // x-coordinate (scaled in meters)
   float y;           // y-coordinate (scaled in meters)
   float theta_rad;   // heading (radians) wrapped into [-pi, pi]

};

/**
 * @brief Specifies motion for a 2D planar robot as robot-frame velocities.
 */
struct VelocityCommand2D {
   float vx_mps;      // Robot-frame "forward" linear velocity (meters per second)
   float wz_radps;    // Robot-frame angular velocity (radians per second)
};

/**
 * @brief scoped enum for handling non-observation landmarks
 */
enum class NONE_OBS_LM {prediction = -1, unknown_lm = -2};

/** An observation of a landmark as viewed from the robot's perspective. */
struct Observation2D {
   float range_m;     // Range from the robot to the landmark (meters)
   float bearing_rad; // Bearing in the robot's frame to the landmark (radians)

   /**
    * @brief Stores the landmark signature corresponding to the observation.
    *    Set to std::nullopt to represent an unknown correspondence variable.
    *
    * TODO: When DA is unknown, each particle's correspondence may differ.
    *    Populating/clearing this field may be more trouble than it's worth...
    */
   std::optional<int> landmarkID;

   /**
    * @brief overloaded difference operator for residual calculation
    * @return an eigen column vector representing the residual
    */
   Eigen::Vector2f operator-(const struct Observation2D& rhs) const {
      float range_diff = this->range_m - rhs.range_m;
      float bearing_diff = this->bearing_rad - rhs.bearing_rad;

      return Eigen::Vector2f(range_diff, bearing_diff);
   }
};
