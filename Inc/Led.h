/*
 * Created on Mon Oct 02 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef LED_H_
#define LED_H_

// Defines
#define IFLED1_Pin GPIO_PIN_10
#define IFLED1_GPIO_Port GPIOB
#define IFLED2_Pin GPIO_PIN_0
#define IFLED2_GPIO_Port GPIOB
#define IFLED3_Pin GPIO_PIN_5
#define IFLED3_GPIO_Port GPIOC
#define IFLED4_Pin GPIO_PIN_4
#define IFLED4_GPIO_Port GPIOC

#include <array>
#include <utility>
#include "stm32f1xx_hal.h"

enum class LedNumber : int { ONE, TWO, THREE, FOUR };

class Led {
 private:
  std::array<std::pair<GPIO_TypeDef *, uint16_t>,4> gpio_pins;
  static Led* instance;
  Led();
  void init();

 public:
  ~Led();
  static Led* getInstance();
  void AllOn();
  void AllOff();
  void On(LedNumber num);
  void Off(LedNumber num);
};

#endif