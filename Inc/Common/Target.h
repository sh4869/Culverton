#ifndef TARGET_H_
#define TARGET_H_

#include "Position.h"
#include "Velocity.h"

struct Target {
    enum class TargetVariety { STRAIGHT, PIVOTTURN, STOP };
    TargetVariety variety;
    Velocity ve;
    Position pos;
    Target(TargetVariety _variety = TargetVariety::STOP,
           Velocity _v = { 0, 0 },
           Position _pos = { 0, 0 })
        : variety(_variety), ve(_v), pos(_pos) {}
};

#endif