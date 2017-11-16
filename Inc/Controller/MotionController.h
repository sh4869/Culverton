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
    static std::shared_ptr<MotionController> GetInstance();
    std::shared_ptr<TargetGenerator> targeter;
    void Enable();
    void Disable();
    const bool& IsEnable();
private:
    static std::shared_ptr<MotionController> instance;
    constexpr static char CVGeneratorSize = 4;
    constexpr static float T = 0.001f;
    std::unique_ptr<SpeedCVGenerator> speedcvgen;
    std::unique_ptr<AngularVelocityCVGenerator> angspeedcvgen;
    std::shared_ptr<StateEstimator> estimator;
    bool enable;
    static Motor *motor;
};


#endif