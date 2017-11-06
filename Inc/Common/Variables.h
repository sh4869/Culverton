#ifndef VARIABLES_H_
#define VARIABLES_H_

// Include Math Constants
#include "arm_math.h"

// 物理定数系
/**
 * ギア比
 */ 
static constexpr float GearRatio = 8.0F / 42.0F;
/**
 * タイヤ半径
 */ 
static constexpr float WheelRadius = 27.0F / 2.0F;
/**
 * エンコーダの1回転あたりのパルス数
 */
static constexpr float EncoderPulse = 1024.0F;

// コース系
static constexpr float WallWidth = 180.0F; // mm
static constexpr float PoleWidth = 12.0F; // mm
#endif