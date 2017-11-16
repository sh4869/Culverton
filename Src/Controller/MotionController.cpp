#include "MotionController.h"

std::shared_ptr<MotionController> MotionController::instance = nullptr;

MotionController::MotionController()
    : estimator(std::make_shared<StateEstimator>(T)),
      targeter(std::make_shared<TargetGenerator>()),
      enable(false) {
    // TODO : 何も考えてない
    PIDParams p(100.0f, 3.4f, 0.0f);
    speedcvgen = std::unique_ptr<SpeedCVGenerator>(new SpeedCVGenerator(p, estimator, targeter));
    angspeedcvgen = std::unique_ptr<AngularVelocityCVGenerator>(new AngularVelocityCVGenerator(p, estimator, targeter));
}

std::shared_ptr<MotionController> MotionController::GetInstance() {
    if (instance == nullptr) {
        instance = std::make_shared<MotionController>();
    }
    return instance;
}

void MotionController::Update() {
    Motor* motor = Motor::GetInstance();
    // 推定機の更新
    estimator->Update();
    // Targetの更新
    targeter->Update();
    // 処理の実行
    switch (targeter->GetTarget().variety) {
        case Target::TargetVariety::STRAIGHT: {
            const auto speedcv = speedcvgen->Update();
            const auto angspeedcv = angspeedcvgen->Update();
            motor->SetDuty(MotorPosition::RIGHT, speedcv + angspeedcv);
            motor->SetDuty(MotorPosition::LEFT, speedcv - angspeedcv);
            break;
        }
        case Target::TargetVariety::PIVOTTURN:
            break;
        case Target::TargetVariety::STOP:{
            // 止める
            const MotorDuty duty = {0,0};
            motor->SetDuty(duty);
            break;
        }
        default:
            const MotorDuty duty = {0,0};
            motor->SetDuty(duty);
            motor->Stop(MotorPosition::RIGHT);
            motor->Stop(MotorPosition::LEFT);
            break;
    }
    
}

void MotionController::SetMotion(Motion::MotionBase* _motion) {
    targeter->SetMotion(_motion);
}

void MotionController::Enable() {
    enable = true;
}

void MotionController::Disable() {
    enable = false;
}

const bool& MotionController::IsEnable() {
    return enable;
}