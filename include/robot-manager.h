/**
 * @file robot-manager.h
 * @brief Defines RobotManager and its derived classes for robot-specific code */

#pragma once

#include "core-structs.h"
#include "math-util.h"

class RobotManager {
public:
    virtual float sampleIMU() = 0;
    virtual struct Observation2D sampleLandMark() = 0;
    virtual struct VelocityCommand2D sampleControl() = 0;
};
