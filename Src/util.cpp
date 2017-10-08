#include "util.h"

void Util::Delay(__IO uint32_t count) {
  for (; count != 0; count--)
    ;
}
void GPIO::On(GPIOPin pin){
    HAL_GPIO_WritePin(pin.first,pin.second,GPIO_PIN_SET);
}
void GPIO::Off(GPIOPin pin){
    HAL_GPIO_WritePin(pin.first,pin.second,GPIO_PIN_RESET);
}