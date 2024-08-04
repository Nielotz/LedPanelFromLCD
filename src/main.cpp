#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>

namespace pinout {
    namespace bl_en {
        constexpr int LED_STRIP_FIRST = 3;   // P4:    ^
        constexpr int LED_STRIP_SECOND = 2;  // P3: |
        constexpr int LED_STRIP_THIRD = 2;   // P3:     |
        constexpr int LED_STRIP_FOURTH = 5;  // P0:   _
    }
    constexpr int BL_PWM = 6;                // P1
    constexpr int DIMMING = 7;               // P2

    void configure() {
        pinMode(BL_PWM, OUTPUT);
        pinMode(DIMMING, INPUT);

        pinMode(bl_en::LED_STRIP_FIRST, OUTPUT);
        pinMode(bl_en::LED_STRIP_SECOND, OUTPUT);
        pinMode(bl_en::LED_STRIP_THIRD, OUTPUT);
        pinMode(bl_en::LED_STRIP_FOURTH, OUTPUT);
    }
}

/**
 * @brief: map x from in_min to in_max to range out_min to out_max. Analogous to map from Arduino.
 */
int mapFloatToInt(float x, float in_min, float in_max, int out_min, int out_max) {
    return static_cast<int>((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

namespace dimming {
    namespace potentiometer {
        constexpr float ADC_RANGE = 1024;  // analogRead range
        constexpr float VOLTAGE_RANGE = 5;
        constexpr float VOLTAGE_MIN = 0.15;  // Bottom voltage - everything below will be read as 0
        constexpr float VOLTAGE_MAX = 4.5;  // Upper voltage - everything above will be read as VOLTAGE_RANGE
        constexpr int ADC_READ_MIN = static_cast<int>(VOLTAGE_MIN / VOLTAGE_RANGE * ADC_RANGE);  // Adjust 0 to match VOLTAGE_MIN.
        constexpr int ADC_READ_MAX = static_cast<int>(VOLTAGE_MAX / VOLTAGE_RANGE * ADC_RANGE);  // Adjust 1024 to match VOLTAGE_MAX.

        /**
         * @brief: Parse read brightness value.
         * @retval: Target brightness: value scaled to range: <0 - 1024).
         */
        inline int parseTargetBrightness(float readValue) {
            if (readValue > VOLTAGE_MAX)
                readValue = VOLTAGE_MAX;
            else if (readValue < VOLTAGE_MIN)
                readValue = VOLTAGE_MIN;

            return mapFloatToInt(readValue, VOLTAGE_MIN, VOLTAGE_MAX, ADC_READ_MIN, ADC_READ_MAX);
        }
    }
}

inline int readTargetBrightness(int mapMin, int mapMax) {
    /**
     * @brief: Read target brightness set by user.
     * @retval: Target brightness: value scaled from range <0, 1024) (ADC resolution) to <mapMin - mapMax).
     */
    int potentiometerValue = analogRead(pinout::DIMMING);
    int targetBrightness = dimming::potentiometer::parseTargetBrightness(potentiometerValue);
    return static_cast<int>(map(targetBrightness, 0, 1024, mapMin, mapMax));
}


//void exitWithErrorLed(int *pulsesMs, int pulsesLength, int msBetweenPulses) {
//    if (DEBUG_USING_LED) {
//        for (int i = 0; i < pulsesLength; i++) {
//            digitalWrite(LED_BUILTIN, HIGH);
//            delay(pulsesMs[i]);
//            digitalWrite(LED_BUILTIN, LOW);
//            delay(msBetweenPulses);
//        }
//    }
//    exit(0);
//}

void setup() {
    pinout::configure();
}


void loop() {
    int targetBrightness = readTargetBrightness(0, 255);

    analogWrite(pinout::BL_PWM, targetBrightness);
    analogWrite(LED_BUILTIN, targetBrightness);

    _delay_ms(100);
}