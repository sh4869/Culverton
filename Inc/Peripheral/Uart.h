#ifndef UART_H_
#define UART_H_

#include "stm32f1xx_hal.h"

class Uart {
private:
    static Uart* instance;
    Uart() = default;
    void init();
    UART_HandleTypeDef huart;
public:
    static Uart* GetInstance();
    void Transmit(const char *);
};

#endif