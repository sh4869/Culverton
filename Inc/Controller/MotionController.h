#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include <array>
#include <memory>
#include "AngularVelocityCVGenerator.h"
#include "SpeedCVGenerator.h"
#include "Motor.h"
#include "StateEstimator.h"
#include "TargetGenerator.h"

class MotionController {
public:
    MotionController();
    ~MotionController() = default;
    void Update();
    void SetMotion(Motion::MotionBase *_motion);
private:
    constexpr static char CVGeneratorSize = 4;
    constexpr static float T = 0.001f;
    std::unique_ptr<SpeedCVGenerator> speedcvgen;
    std::unique_ptr<AngularVelocityCVGenerator> angspeedcvgen;
    std::shared_ptr<StateEstimator> estimator;
    std::shared_ptr<TargetGenerator> targeter;
    Motor *motor;
};


#endif