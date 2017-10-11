#include "Timer.h"

Timer* Timer::instance = nullptr;

Timer::Timer() {}

void Timer::init() {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 63;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 5000;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
}

Timer* Timer::GetInstance(){
    if(instance == nullptr){
        instance = new Timer();
        instance->init();
    }
    return instance;
}

void Timer::Start(){
    if(HAL_TIM_Base_GetState(&htim4) != HAL_TIM_STATE_BUSY){
        HAL_TIM_Base_Start_IT(&htim4);
    }
}

void Timer::Stop(){
    HAL_TIM_Base_Stop_IT(&htim4);
}