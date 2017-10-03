#include "Switch.h"

#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_10
#define SW2_GPIO_Port GPIOC

Switch * Switch::instance = nullptr;

Switch::Switch() {}

void Switch::init() {
    gpio_pins = {std::make_pair(SW1_GPIO_Port, SW1_Pin), std::make_pair(SW2_GPIO_Port, SW2_Pin)};
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = SW2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SW1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);
}

Switch::~Switch() { delete instance; }

Switch* Switch::getInstance() {
    if(instance == nullptr){
        instance = new Switch();
        instance->init();
    }
    return instance;
}

bool Switch::isPressed(SwitchNumber number){
    int index = static_cast<int>(number);
    // TODO チャタリング対策したほうが良さそう？
    if(HAL_GPIO_ReadPin(gpio_pins[index].first,gpio_pins[index].second) == GPIO_PIN_SET){
        return true;
    }
    return false;
}