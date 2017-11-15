#include "MotionController.h"


MotionController::MotionController()
    : estimator(std::make_shared<StateEstimator>(T)),
      targeter(std::make_shared<TargetGenerator>()),
      motor(Motor::GetInstance()) {
    // TODO : 何も考えてない
    PIDParams p(100.0f,3.4f,0.0f); 
    speedcvgen = std::make_unique<SpeedCVGenerator>(p, estimator, targeter);
    angspeedcvgen = std::make_unique<AngularVelocityCVGenerator>(p, estimator, targeter);
}

void MotionController::Update() {
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

void MotionController::SetMotion(Motion::MotionBase *_motion){
    targeter->SetMotion(_motion);
}