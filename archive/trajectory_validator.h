#ifndef TRAJECTORY_VALIDATOR_H
#define TRAJECTORY_VALIDATOR_H

#include <Arduino.h>
#include "planner.h"

class WorkPlane;

struct ValidationResult {
    bool ok{true};
    int failIndex{-1};
    String reason{"OK"};
};

[[nodiscard]] ValidationResult validateTrajectory(const Planner::Job& job,
                                                  const WorkPlane* workPlane = nullptr);

#endif // TRAJECTORY_VALIDATOR_H
