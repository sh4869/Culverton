#ifndef MOTION_H_
#define MOTION_H_

#include <memory>

#include "Target.h"
#include "Odometry.h"

namespace Motion {
    class MotionBase {
    protected:
        bool is_finish;
    public:
        virtual ~MotionBase() = default;
        virtual Target next() = 0;
        bool Finished() { return is_finish; };
    };
    class Straight : public MotionBase {
    public:
        Straight(float T,float L,float v_start,float v_target,float v_end,float accel);
        ~Straight() = default;
        Target next() override;
    private:
        float T;
        float L;
        float v_start,v_target,v_end;
        float t1,t2,t3;
        float accel;
        int32_t cnt;
        int32_t basecount;
        std::unique_ptr<Odometry> odometry;
    };
}

#endif