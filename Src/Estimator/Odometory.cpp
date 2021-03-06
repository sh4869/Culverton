#include "Odometry.h"
#include <cmath>

/**
 * @brief コンストラクタ
 *
 * @param _T 周期
 */
Odometry::Odometry(float _T)
    : T(_T), pos(), prev_x(0), prev_y(0), prev_omega(0), initialized(true) {}

/**
 * @brief Positionをセットします
 *
 * @param p セットするPosition
 */
void Odometry::SetPositon(Position p) { pos = p; }

/**
 * @brief 現在推定位置を返します
 *
 * @return Position& 現在推定位置
 */
const Position& Odometry::GetPosition() const { return pos; }

/**
 * @brief オドメトリをアップデートします
 *
 * @param v Velocity
 */
void Odometry::Update(const Velocity &v) {
    const float vx = -std::sin(pos.theta) * v.v;
    const float vy = std::cos(pos.theta) * v.v;
    if (!initialized) {
        prev_x = vx;
        prev_y = vy;
        prev_omega = v.omega;
        initialized = true;
    }
    pos.x += (vx + prev_x) / 2.0 * T;
    pos.y += (vy + prev_y) / 2.0 * T;
    pos.theta += (v.omega + prev_omega) / 2.0 * T;
    prev_x = vx;
    prev_y = vy;
    prev_omega = v.omega;
}

/**
 * @brief リセットします
 * 
 */
void Odometry::Reset() {
    prev_x = prev_y = prev_omega = 0.0f;
    pos.x = pos.y = pos.theta = 0;
    initialized = false;
}