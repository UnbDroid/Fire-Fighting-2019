#ifndef REFLECTION_H_
#define REFLECTION_H_

#include "Arduino.h"

// Class created to represent the reflection sensor
class Reflection {
private:
  // Number of samples used when filtering the sensor
  const static int SAMPLE_NUM = 5;

  int     pin;    // digital input pin
  bool    color;  // saves the last color read after filtering

  // Private method used in update() to sort an array of samples
  void sort (bool *sample);
public:
  // Constants for the colors
  const static bool BLACK = true;
  const static bool WHITE = false;

  // Constructor method. Receives the digital input pin number
  Reflection(int _pin);

  // Reads the sensor and updates the last color seen
  void update();

  // Returns the last color seen by the sensor
  bool getColor();
};

#endif // REFLECTION_H_