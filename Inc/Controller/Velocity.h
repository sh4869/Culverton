#ifndef TRACKING_TARGET_H_
#define TRACKING_TARGET_H_

#include "arm_math.h"
/**
 * Velocity
 */
struct Velocity {
    float v;
    float omega;
    /**
     * @brief コンストラクタ
     *
     * @param 0.0f 速度
     * @param 0.0f 角速度
     */
    Velocity(float _v = 0.0f, float _omega = 0.0f) : v(_v), omega(_omega){};
    Velocity operator+(const Velocity other) const {
        return Velocity(this->v + other.v, this->omega + other.omega);
    }
    Velocity operator-(const Velocity other) const {
        return Velocity(this->v - other.v, this->omega - other.omega);
    }
};

#endif