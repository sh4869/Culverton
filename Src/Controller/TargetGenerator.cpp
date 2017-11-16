#include "TargetGenerator.h"

void TargetGenerator::Update() {
    // nullチェック
    if (!motions.empty()) {
        if (motions.front()->Finished()) {
            motions.pop();
        }
        if (!motions.empty()) {
            currentTarget = motions.front()->next();
        } else {
            Target none;
            none.variety = Target::TargetVariety::STOP;
            none.pos = Position();
            none.ve = Velocity();
            currentTarget = none;
        }
    } else {
        Target none;
        none.variety = Target::TargetVariety::STOP;
        none.pos = Position();
        none.ve = Velocity();
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

bool TargetGenerator::HasMotion() {
    return !motions.empty();
}