#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <cfloat>

struct PIDParams {
    float kp;
    float ki;
    float kd;
    float kr;
    float T;
    PIDParams(float _kp = 0.0f,
              float _ki = 0.0f,
              float _kd = 0.0f,
              float _kr = 0.0f,
              float _T = 0.001f)
        : kp(_kp), ki(_ki), kd(_kd), kr(_kr), T(_T) {}
};

class PIDController {
public:
    PIDParams param;
    PIDController();
    PIDController(const PIDParams& _param);
    ~PIDController();
    float Update(float measured, float reference);
    void Reset();

    void SetParams(const PIDParams& _param);
    const PIDParams& GetParams();

private:
    float prev_error = 0.0f;
    float error_sum = 0.0f;
    bool is_first = true;
};

#endif