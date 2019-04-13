#include "reflection.h"

// Constructor method. Already sets the pin mode
Reflection::Reflection(int _pin) {
    pin = _pin;
    pinMode(pin, INPUT);
}

// Bubble sort algorythm used to sort the array of samples
void Reflection::sort (bool *sample) {
    int k, j;
    bool aux;

    for (k = 1; k < SAMPLE_NUM; k++) {
        for (j = 0; j < SAMPLE_NUM - 1; j++) {
            if (sample[j] > sample[j + 1]) {
                aux           = sample[j];
                sample[j]     = sample[j + 1];
                sample[j + 1] = aux;
            }
        }
    }
}

// Reads the sensor and updates the last seen color. Filters by median.
void Reflection::update() {
    bool sample[SAMPLE_NUM]; // array of samples

    // Fills the array
    for(int i = 0; i < SAMPLE_NUM; i++)
        sample[i] = digitalRead(pin);

    sort(sample);

    // Takes the median of all samples and updates the last color
    color = sample[(SAMPLE_NUM + 1)/2];
}

// Returns the last seen color
bool Reflection::getColor() {
    return this->color;
}

// Prints sensor reading in serial monitor
void Reflection::printColor() {
    this->update();
    Serial.println(this->getColor());
}