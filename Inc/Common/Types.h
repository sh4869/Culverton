#ifndef TYPES_H_
#define TYPES_H_

#include <utility>
#include "stm32f1xx_hal.h"

using GPIOPin = std::pair<GPIO_TypeDef*,uint16_t>;

static constexpr auto GPIOPinPair = std::make_pair<GPIOPin::first_type,GPIOPin::second_type>;

#endif