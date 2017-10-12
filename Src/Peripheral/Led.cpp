#include "Led.h"
#include <map>
#include "stm32f1xx_hal.h"

// Defines
#define IFLED1_Pin GPIO_PIN_10
#define IFLED1_GPIO_Port GPIOB
#define IFLED2_Pin GPIO_PIN_0
#define IFLED2_GPIO_Port GPIOB
#define IFLED3_Pin GPIO_PIN_5
#define IFLED3_GPIO_Port GPIOC
#define IFLED4_Pin GPIO_PIN_4
#define IFLED4_GPIO_Port GPIOC

Led* Led::instance = nullptr;

Led::Led() {}

void Led::init() {
    gpio_pins = {
        GPIOPinPair(IFLED1_GPIO_Port, IFLED1_Pin),GPIOPinPair(IFLED2_GPIO_Port, IFLED2_Pin),
        GPIOPinPair(IFLED3_GPIO_Port, IFLED3_Pin), GPIOPinPair(IFLED4_GPIO_Port, IFLED4_Pin),
    };
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = IFLED4_Pin | IFLED3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
    GPIO_InitStruct.Pin = IFLED2_Pin | IFLED1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
Led::~Led() { delete instance; }

Led* Led::GetInstance() {
    if (instance == nullptr) {
        instance = new Led();
        instance->init();
    }
    return instance;
}

void Led::AllOn() {
    for (GPIOPin pin : gpio_pins) {
        GPIO::On(pin);
    }
}

void Led::AllOff() {
    for (GPIOPin pin : gpio_pins) {
        GPIO::Off(pin);
    }
}

void Led::On(const LedNumber num) {
    int index = static_cast<int>(num);
    GPIO::On(gpio_pins[index]);
}

void Led::Off(const LedNumber num) {
    int index = static_cast<int>(num);
    GPIO::Off(gpio_pins[index]);
}

void Led::OnOnly(const LedNumber num) {
    AllOff();
    Led::On(num);
}