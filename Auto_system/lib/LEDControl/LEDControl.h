#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

/**
 * @brief Class to control an RGB LED.
 * 
 * This class provides methods to set the color of an RGB LED using PWM.
 */
class LEDControl {
public:
    /**
     * @brief Construct a new LEDControl object.
     * 
     * @param pinR PWM pin for Red
     * @param pinG PWM pin for Green
     * @param pinB PWM pin for Blue
     * @param commonAnode Set to true if using a common anode LED (active low). Default is false (common cathode, active high).
     */
    LEDControl(int pinR, int pinG, int pinB, bool commonAnode = false);

    /**
     * @brief Initialize the LED pins.
     */
    void begin();

    /**
     * @brief Set the color of the RGB LED.
     * 
     * @param r Red intensity (0-255)
     * @param g Green intensity (0-255)
     * @param b Blue intensity (0-255)
     */
    void setColor(int r, int g, int b);

    /**
     * @brief Turn off the LED.
     */
    void off();

    // Preset colors
    void red();
    void green();
    void blue();
    void yellow();
    void cyan();
    void magenta();
    void white();

private:
    int _pinR, _pinG, _pinB;
    bool _commonAnode;
    void _write(int pin, int value);
};

#endif
