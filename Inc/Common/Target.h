#ifndef TARGET_H_
#define TARGET_H_

#include "Velocity.h"
#include "Position.h"

struct Target {
    enum class TargetVariety { STRAIGHT, PIVOTTURN, STOP };
    TargetVariety variety;
    Velocity ve;
    Position pos;
};

#endif