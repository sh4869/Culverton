#include "Motion.h"
#include "Timer.h"

namespace Motion {
    Straight::Straight(float T, float L, float v_start, float v_target, float v_end, float accel)
        : T(T),
          L(L),
          v_start(v_start),
          v_target(v_target),
          v_end(v_end),
          accel(accel),
          odometry(std::unique_ptr<Odometry>(new Odometry(T))),
          is_finish(false) {
        cnt = 0;
        odometry->Reset();
        basecount = Timer::GetCount();
        // とりあえず距離内でに加速しきるかの計算
        float _tstart = (v_target - v_start) / (accel * T);
        float _tend = (v_target - v_end) / (accel * T);
        // 加速しきらない場合なのでしんどいね～～～となります．はい．
        if (L < ((v_target - v_start) * _tstart * 0.5 * T + v_start * _tstart * T) +
                        ((v_target - v_end) * _tend * 0.5 * T + v_end * _tend * T)) {
            // TODO: ここの正しい値を計算
        } else {
            // 加速仕切ることがわかるならt1,t2,t3を計算
            t1 = _tstart;
            const float L2 =
                    L - (((v_target - v_start) * _tstart * 0.5 * T + v_start * _tstart * T) +
                         ((v_target - v_end) * _tend * 0.5 * T + v_end * _tend * T));
            t2 = (L2 / v_target / T) + t1;
            t3 = t2 + _tend;
        }
    }
    Target Straight::next() {
        cnt++;
        float v;
        if (cnt < t1) {
            // 加速中
            v = v_start + accel * cnt * T;
        } else if (cnt < t2) {
            // 最高速度
            v = v_target;
        } else if (cnt < t3) {
            // 減速中
            v = v_end + accel * (t3 - cnt) * T;
        } else {
            is_finish = true;
            v = v_end;
        }
        // 角速度は直進の場合 0
        const Velocity vel(v,0);
        odometry->Update(vel);
        const Position pos = odometry->GetPosition();
        const Target target = { Target::TargetVariety::STRAIGHT, vel, pos };
        return target;
    }
    bool Straight::Finished() {
        return is_finish;
    }
}  // namespace Motion