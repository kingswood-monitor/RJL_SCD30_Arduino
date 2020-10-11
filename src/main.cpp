#include <Arduino.h>

#include <RJL_SCD30_Arduino.h>

SCD30 airSensor;

uint16_t air_pressure = 900;
uint16_t temperature_offset = 0.5;

void setup() {

    Serial.begin(115200);
    pinMode(17, OUTPUT);

    delay(1000);

    airSensor.begin(Wire);
    // airSensor.setMeasurementInterval(2);
    Serial.println("Started");

    airSensor.activateAutomaticSelfCalibration();

    // airSensor.setTemperatureOffset(0);

    // Serial.print("getAutomaticSelfCalibrationStatus: ");
    // Serial.println(airSensor.getAutomaticSelfCalibrationStatus());

    // Serial.print("getForcedCalibrationValue: ");
    // Serial.println(airSensor.getForcedCalibrationValue());

    // Serial.print("Activating automatic self calibration...");
    // bool success = airSensor.activateAutomaticSelfCalibration();
    // success ? Serial.println("OK") : Serial.println("Fail");

    // Serial.print("getAutomaticSelfCalibrationStatus=");
    // Serial.println(airSensor.getAutomaticSelfCalibrationStatus());

    // Serial.print("Deactivating...");
    // success = airSensor.deactivateAutomaticSelfCalibration();
    // success ? Serial.println("OK") : Serial.println("Fail");

    // Serial.print("getAutomaticSelfCalibrationStatus: ");
    // Serial.println(airSensor.getAutomaticSelfCalibrationStatus());
}

void loop() {

    if (airSensor.dataAvailable())
    {
        // Serial.print("CO2:");
        // Serial.println(airSensor.getCO2());

        // Serial.print("Temp: ");
        // Serial.println(airSensor.getTemperature());

        // Serial.print("Hum:");
        // Serial.println(airSensor.getHumidity());
    }
    digitalWrite(17, HIGH);
    delay(1000);
    digitalWrite(17, LOW);
    delay(1000);
}
