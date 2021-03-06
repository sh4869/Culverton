#ifndef BUZZER_H_
#define BUZZER_H_

#include "stm32f1xx_hal.h"

class Buzzer {
private:
    static Buzzer* instance;
    Buzzer() = default;
    void init();
    TIM_HandleTypeDef htim5;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    static constexpr uint32_t timechannel = TIM_CHANNEL_3;
public:
    static Buzzer* GetInstance();
    void On(uint32_t pulse);
    void Off();
};

#endif