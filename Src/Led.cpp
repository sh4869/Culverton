#include "Led.h"
#include <map>

Led* Led::instance = nullptr;

std::array<std::pair<GPIO_TypeDef*, uint16_t>, 4> pin_map = {
    std::make_pair(IFLED1_GPIO_Port, IFLED1_Pin), std::make_pair(IFLED2_GPIO_Port, IFLED2_Pin),
    std::make_pair(IFLED3_GPIO_Port, IFLED3_Pin), std::make_pair(IFLED4_GPIO_Port, IFLED4_Pin),
};

Led::Led() {}

void Led::init(){
    gpio_pins = pin_map;
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
Led::~Led() {
    delete instance;
}

Led* Led::getInstance(){
    if(instance == nullptr){
        instance = new Led();
        instance->init();
    }
    return instance;
}

void Led::AllOn() {
    for (uint i = 0; i < pin_map.size(); i++) {
        HAL_GPIO_WritePin(gpio_pins[i].first, gpio_pins[i].second, GPIO_PIN_SET);
    }
}

void Led::AllOff() {
    for (uint i = 0; i < pin_map.size(); i++) {
        HAL_GPIO_WritePin(gpio_pins[i].first, gpio_pins[i].second, GPIO_PIN_SET);
    }
}

void Led::On(LedNumber num) {
    int index = static_cast<int>(num);
    HAL_GPIO_WritePin(gpio_pins[index].first,gpio_pins[index].second,GPIO_PIN_SET);
}

void Led::Off(LedNumber num){
    int index = static_cast<int>(num);
    HAL_GPIO_WritePin(gpio_pins[index].first,gpio_pins[index].second,GPIO_PIN_RESET);
}