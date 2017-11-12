#ifndef MOTOR_H_
#define MOTOR_H_


#include <cstdint>
#include "stm32f1xx_hal.h"
#include "util.h"
#include "Types.h"

enum class MotorDirection : std::uint8_t { FRONT, BACK };
enum class MotorPosition : std::uint8_t { RIGHT, LEFT };

struct MotorPins {
    GPIOPin In1Pin,In2Pin;
};

class Motor {
private:
    static Motor* instance;
    Motor();
    void init();

    TIM_HandleTypeDef htim8;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    uint32_t rightChannel = TIM_CHANNEL_2, leftChannel = TIM_CHANNEL_1;
    GPIOPin stbyPin;
    MotorPins rightPins,leftPins;
    bool is_standby = false;

public:
    static Motor* GetInstance();
    bool IsStandby();
    void SetStandby();
    void UnSetStandby();
    void SetDirection(MotorPosition pos, MotorDirection dir);
    void Stop(MotorPosition pos);
    void Start(MotorPosition pos,uint32_t pulse);
    void UpdatePWM(MotorPosition pos, uint32_t pulse);
};

#endif