#ifndef MOUSESYSTEM_H_
#define MOUSESYSTEM_H_


class MouseSystem {
private:
    void init();
public:
    MouseSystem();
    ~MouseSystem();
    void StartMouse();
    void BatteryCheck();
};

#endif