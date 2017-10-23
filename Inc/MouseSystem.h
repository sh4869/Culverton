#ifndef MOUSESYSTEM_H_
#define MOUSESYSTEM_H_

#include "BatteryMonitor.h"
#include "Buzzer.h"
#include "Led.h"
#include "Sensor.h"
#include "Uart.h"

enum class MouseMode : int { NONE, SCAN, SET_SENSOR, RUN };

class MouseSystem {
private:
    // Variables
    Led *led;
    Buzzer *buzzer;
    Sensor *sensor;
    Uart *uart;
    BatteryMonitor *bm;

    void init();
    void initPeripheral();
    
public:
    MouseSystem();
    ~MouseSystem();
    void StartMouse();
    void BatteryCheck();
};

#endif