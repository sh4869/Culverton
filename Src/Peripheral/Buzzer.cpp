#include "Buzzer.h"

Buzzer* Buzzer::instance = nullptr;

void Buzzer::init() {
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 6400 - 1;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim5);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, timechannel);
}

Buzzer* Buzzer::GetInstance() {
    if (instance == nullptr) {
        instance = new Buzzer();
        instance->init();
    }
    return instance;
}

void Buzzer::On(uint32_t pulse) {
    sConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC,timechannel);
    HAL_TIM_PWM_Start(&htim5, timechannel);
}

void Buzzer::Off(){
    HAL_TIM_PWM_Stop(&htim5, timechannel);
}