#include <Arduino.h>

#include <RJL_SCD30_Arduino_Driver.h>

SCD30Driver driver;

void setup() {

    Serial.begin(115200);
    delay(1000);

    driver.begin(Wire);
    Wire.begin(SDA, SCL);

    Serial.println("Started");

    pinMode(17, OUTPUT);

    Measurement m = driver.readMeasurement();

    Serial.print("CO2: ");

    float co2 = 0;

    memcpy(&co2, &m.co2, sizeof(co2));

    Serial.println(co2);
}

void loop() {
    digitalWrite(17, HIGH);
    delay(100);
    digitalWrite(17, LOW);
    delay(100);
}
