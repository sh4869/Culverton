#ifndef MOTION_H_
#define MOTION_H_

#include <memory>

#include "Target.h"
#include "Odometry.h"

namespace Motion {
    class MotionBase {
    public:
        virtual ~MotionBase() = default;
        virtual Target next() = 0;
        virtual bool Finished() = 0;
    };
    class Straight : public MotionBase {
    public:
        Straight(float T,float L,float v_start,float v_target,float v_end,float accel);
        ~Straight() = default;
        Target next() override;
        bool Finished() override;
    private:
        float T;
        float L;
        float v_start,v_target,v_end;
        float t1,t2,t3;
        float accel;
        int32_t cnt;
        int32_t basecount;
        std::unique_ptr<Odometry> odometry;
        bool is_finish;
    };
}

#endif