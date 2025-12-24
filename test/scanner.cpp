#include <Arduino_Extended.h>

TwoWire i2c3(PB8, PA8);

void setup() {
    Serial.begin();
    i2c3.begin();
    pinMode(PB5, OUTPUT);
    pinMode(PA0, OUTPUT);
}

void loop() {
    i2c_detect(Serial, i2c3, 0x00, 127);
    delay(1000);
}
