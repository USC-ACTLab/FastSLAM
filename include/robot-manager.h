/**
 * @file robot-manager.h
 * @brief Defines RobotManager and its derived classes for robot-specific code */

#pragma once

#include "core-structs.h"
#include "math-util.h"

class RobotManager {
public:
    virtual void sampleIMU() = 0;
    virtual void sampleLandMark() = 0;
    virtual void sampleControl() = 0;
};
