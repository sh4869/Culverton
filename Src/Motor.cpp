#include "Motor.h"

#define MOTOR_LEFT_PWM_Pin GPIO_PIN_6
#define MOTOR_LEFT_PWM_GPIO_Port GPIOC
#define MOTOR_RIGHT_PWM_Pin GPIO_PIN_7
#define MOTOR_RIGHT_PWM_GPIO_Port GPIOC
#define MOTOR_LEFT_IN1_Pin GPIO_PIN_8
#define MOTOR_LEFT_IN1_GPIO_Port GPIOC
#define MOTOR_LEFT_IN2_Pin GPIO_PIN_9
#define MOTOR_LEFT_IN2_GPIO_Port GPIOC
#define MOTOR_RIGHT_IN1_Pin GPIO_PIN_11
#define MOTOR_RIGHT_IN1_GPIO_Port GPIOC
#define MOTOR_RIGHT_IN2_Pin GPIO_PIN_12
#define MOTOR_RIGHT_IN2_GPIO_Port GPIOA
#define MOTOR_STBY_Pin GPIO_PIN_11
#define MOTOR_STBY_GPIO_Port GPIOA

Motor* Motor::instance = nullptr;

Motor::Motor() {}

void Motor::init() {
    stbyPin = GPIOPinPair(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin);
    rightPins.In1Pin = GPIOPinPair(MOTOR_RIGHT_IN1_GPIO_Port, MOTOR_RIGHT_IN1_Pin);
    rightPins.In2Pin = GPIOPinPair(MOTOR_RIGHT_IN2_GPIO_Port, MOTOR_RIGHT_IN2_Pin);
    leftPins.In1Pin = GPIOPinPair(MOTOR_LEFT_IN1_GPIO_Port, MOTOR_LEFT_IN1_Pin);
    leftPins.In2Pin = GPIOPinPair(MOTOR_LEFT_IN2_GPIO_Port, MOTOR_LEFT_IN2_Pin);
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = MOTOR_STBY_Pin | MOTOR_RIGHT_IN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_LEFT_IN1_Pin | MOTOR_LEFT_IN2_Pin | MOTOR_RIGHT_IN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 63;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 999;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim8);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);
}

void Motor::updatePWM(uint32_t channel, uint32_t pulse) {
    sConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, channel);
    HAL_TIM_PWM_Start(&htim8, channel);
}

Motor* Motor::GetInstance() {
    if (instance == nullptr) {
        instance = new Motor();
        instance->init();
    }
    return instance;
}

bool Motor::IsStandby() { return is_standby; }

void Motor::SetStandby() {
    GPIO::On(stbyPin);
    is_standby = true;
}

void Motor::UnSetStandby() {
    GPIO::Off(stbyPin);
    is_standby = false;
}

void Motor::SetDirection(MotorPosition pos, MotorDirection dir) {
    MotorPins pins;
    switch (pos) {
        case MotorPosition::RIGHT:
            pins = rightPins;
            break;
        case MotorPosition::LEFT:
            pins = leftPins;
            break;
    }
    switch (dir) {
        case MotorDirection::FRONT:
            GPIO::On(pins.In1Pin);
            GPIO::Off(pins.In2Pin);
            break;
        case MotorDirection::BACK:
            GPIO::Off(pins.In1Pin);
            GPIO::On(pins.In2Pin);
            break;
    }
}

// あくまでPWMを止める形で
void Motor::Stop() {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
}

void Motor::Start(uint32_t pulse) {
    if (!is_standby) {
        SetStandby();
    }
    SetDirection(MotorPosition::RIGHT, MotorDirection::FRONT);
    SetDirection(MotorPosition::LEFT, MotorDirection::FRONT);
    updatePWM(rightChannel, pulse);
    updatePWM(leftChannel, pulse);
}