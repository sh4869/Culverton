#ifndef SPEEDESTIMATOR_H_
#define SPEEDESTIMATOR_H_

#include <stdint.h>
#include "Velocity.h"
#include "Encoder.h"

class SpeedEstimator {
public:
    SpeedEstimator(float _T);
    ~SpeedEstimator() = default;
    void Update(const EncoderVelocity &enc);
    const Velocity& GetVelocity() const;
private:
    float T;
    EncoderVelocity prev_enc;
    Velocity ve;
};

#endif