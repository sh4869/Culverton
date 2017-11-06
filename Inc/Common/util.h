#ifndef UTIL_H_
#define UTIL_H_

#include "Types.h"
#include "stm32f1xx_hal.h"
#include <utility>

namespace Util {
    void Delay(__IO uint32_t count);
}

namespace GPIO {
    void On(GPIOPin pin);
    void Off(GPIOPin pin);
}

#endif