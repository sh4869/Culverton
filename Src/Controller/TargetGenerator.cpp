#include "TargetGenerator.h"

void TargetGenerator::Update() {
    if (motions.front()->Finished()) {
        motions.pop();
    }
    if (!motions.empty()) {
        currentTarget = motions.front()->next();
    } else {
        currentTarget = none;
    }
}

const Target& TargetGenerator::GetTarget() {
    return currentTarget;
}

const Position& TargetGenerator::GetTargetPosition() {
    return currentTarget.pos;
}

const Velocity& TargetGenerator::GetTargetVelocity() {
    return currentTarget.ve;
}

void TargetGenerator::SetMotion(Motion::MotionBase* _motion) {
    motions.push(_motion);
}