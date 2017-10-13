#include "MouseSystem.h"
#include "Led.h"
#include "Sensor.h"
#include "adc.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

MouseSystem::MouseSystem() { init(); }

MouseSystem::~MouseSystem() { delete this; }

void MouseSystem::init() {
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_TIM8_Init();
}

// パフォーマンス的な
void MouseSystem::StartMouse() {
    static Led* led = Led::GetInstance();
    static Sensor* sensor = Sensor::GetInstance();
    sensor->LedOn(SensorLedNumber::FRONT);
    led->AllOn();
    HAL_Delay(1000);
    sensor->LedOff(SensorLedNumber::FRONT);
    led->AllOff();
    HAL_Delay(100);
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 4; i++) {
            led->OnOnly(static_cast<LedNumber>(i));
            HAL_Delay(200);
        }
    }
    led->AllOn();
    HAL_Delay(1000);
    led->AllOff();
    HAL_Delay(500);
}