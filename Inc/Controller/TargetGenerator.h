#ifndef TARGETGENERATOR_H_
#define TARGETGENERATOR_H_

#include <queue>

#include "Position.h"
#include "Velocity.h"
#include "Target.h"
#include "Motion.h"

class TargetGenerator {
public:
    TargetGenerator() = default;
    void Update();
    const Velocity& GetTargetVelocity();
    const Position& GetTargetPosition();
    const Target& GetTarget();
    void SetMotion(Motion::MotionBase *_motion);
    bool HasMotion();
private:
    // もちろん最終的にはqueueにするがとりあえず
    std::queue<Motion::MotionBase*> motions;
    Target currentTarget;
};

#endif