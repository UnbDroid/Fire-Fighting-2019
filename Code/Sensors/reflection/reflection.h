#ifndef REFLECTION_H_
#define REFLECTION_H_

#include "Arduino.h"

// Constants for the colors
#define BLACK true
#define WHITE false

// Class created to represent the reflection sensor
class Reflection {
private:
    // Number of samples used when filtering the sensor
    const static int SAMPLE_NUM = 5;

    int     pin;    // digital input pin
    bool    color;  // saves the last color read after filtering

    // Private method used in update()
    void sort (bool *sample);
public:
    // Public methods for creating and using the sensor object
    Reflection(int _pin);
    void update();
    bool getColor();
};

#endif // REFLECTION_H_