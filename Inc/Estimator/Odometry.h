#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "Position.h"
#include "Velocity.h"

class Odometry {
public:
    Odometry(float _T);
    ~Odometry();
    void Update(const Velocity &v);
    void SetPositon(Position p);
    void Reset();
    const Position& GetPosition();
private:
    const float T;
    Position pos;
    float prev_x,prev_y,prev_omega;
    bool initialized;
};

#endif