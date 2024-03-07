/**
 * @file core-structs.h
 * @brief Defines the basic structs used to implement FastSLAM 1.0.
 */

#pragma once

#include <optional>

/** A point (position only) in the 2D plane. */
struct Point2D {
   float x;           // x-coordinate (scaled in meters)
   float y;           // y-coordinate (scaled in meters)
};

/** A pose (position and orientation) in the 2D plane. */
struct Pose2D {
   float x;           // x-coordinate (scaled in meters)
   float y;           // y-coordinate (scaled in meters)
   float theta_rad;   // heading (radians) wrapped into [-pi, pi]
};

/**
 * @brief A motion command for a 2D planar robot specified as egocentric velocity.
 */
struct VelocityCommand2D {
   float vx_mps;      // Robot-frame "forward" linear velocity (meters per second)
   float wz_radps;    // Robot-frame angular velocity (radians per second)
};

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
};