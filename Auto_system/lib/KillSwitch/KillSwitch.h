#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H

#include <Arduino.h>

class KillSwitch {
public:
    KillSwitch(int pin);
    void begin();
    bool isKilled();
    void kill();
    void reset();
    void printStatus();
private:
    int _pin;
    bool _killed;
};

#endif // KILL_SWITCH_H
