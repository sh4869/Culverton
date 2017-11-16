#include "MouseSystem.h"

#include "adc.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

MouseSystem::MouseSystem() {
    init();
}

void MouseSystem::init() {
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_TIM8_Init();
}

void MouseSystem::initPeripheral() {}

// パフォーマンス的な
void MouseSystem::StartMouse() {
    static Led *led = Led::GetInstance();
    static Buzzer *buzzer = Buzzer::GetInstance();
    // Start Perfomnace
    buzzer->On(2500);
    led->AllOn();
    HAL_Delay(500);
    buzzer->Off();
    led->AllOff();
    HAL_Delay(100);
    for (int i = 0; i < 4; i++) {
        led->OnOnly(static_cast<LedNumber>(i));
        HAL_Delay(200);
    }
    led->AllOn();
    HAL_Delay(1000);
    led->AllOff();
    HAL_Delay(500);
}

void MouseSystem::BatteryCheck() {
    static BatteryMonitor *bm = BatteryMonitor::GetInstance();
    if ((bm->GetValue() * (8.0F / 5.0F)) < 6.0F) {
        char str[1000];
        while ((bm->GetValue() * (8.0F / 5.0F)) < 6.0F) {
            sprintf(str, "%f\n", bm->GetValue() * (8.0F / 5.0F));
            uart->Transmit(str);
            led->AllOn();
            HAL_Delay(1000);
            led->AllOff();
            HAL_Delay(1000);
        }
    }
}