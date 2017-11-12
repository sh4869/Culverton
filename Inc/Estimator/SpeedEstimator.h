#ifndef SPEEDESTIMATOR_H_
#define SPEEDESTIMATOR_H_

#include <stdint.h>
#include "Velocity.h"
#include "Encoder.h"

class SpeedEstimator {
public:
    SpeedEstimator(float _T);
    ~SpeedEstimator() = default;
    void Update(const EncoderValue &enc);
    const Velocity& GetVelocity() const;
private:
    float T;
    EncoderValue prev_enc;
    Velocity ve;
};

#endif