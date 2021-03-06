#ifndef STATE_ESTIMATOR_H_
#define STATE_ESTIMATOR_H_

#include <memory>

#include "Encoder.h"
#include "Odometry.h"
#include "Position.h"
#include "SpeedEstimator.h"
#include "Velocity.h"

class StateEstimator {
public:
    StateEstimator(float _T);
    ~StateEstimator() = default;
    void Update();
    const Velocity& GetVelocity();
    const Position& GetPositon();

private:
    float T;
    std::unique_ptr<SpeedEstimator> speedestimator;
    std::unique_ptr<Odometry> odometry;
    Encoder* encoder;
    Position pos;
    Velocity velocity;
};

#endif