#include "RJL_SCD30_Arduino.h"

/*-----------------------------------------------------------
 * CREATORS
 *----------------------------------------------------------*/

SCD30::SCD30() {
}

/*-----------------------------------------------------------
 * MANIPULATORS
 *----------------------------------------------------------*/

// Start continuous measurement, deactivating ambient pressure compensation.
// If compensation is needed, set subsequently via setAmbientPressureCompensation()
bool SCD30::begin(TwoWire &wirePort) {

    if (_driver.begin(wirePort) != true)
    {
        return false;
    }

    return true;
}

bool SCD30::stop() {
    return _driver.stopContinuousMeasurement();
}

bool SCD30::setTemperatureOffset(float tempOffset) {
    uint16_t tick_offset = tempOffset * 100;
    return _driver.setTemperatureOffset(tick_offset);
}

bool SCD30::setAmbientPressure(uint16_t pressure) {
    return _driver.triggerContinuousMeasurement(pressure);
}

bool SCD30::setMeasurementInterval(uint16_t interval) {
    return _driver.setMeasurementInterval(interval);
}

bool SCD30::setAltitudeCompendation(uint16_t altitude) {
    return _driver.setAltitudeCompensation(altitude);
}

bool SCD30::setForcedCalibrationValue(uint16_t cref) {
    return _driver.setForcedCalibrationValue(cref);
}

bool SCD30::activateAutomaticSelfCalibration() {
    return _driver.activateAutomaticSelfCalibration();
}

bool SCD30::deactivateAutomaticSelfCalibration() {
    bool success = _driver.deactivateAutomaticSelfCalibration();
    Serial.print("In deactivateAutomaticSelfCalibration, result=");
    Serial.println(success);
    
    if (!success)
        return false;
    // Only takes effect after a reset (notes don't say this)
    _driver.softReset();
    return true;
}

bool SCD30::softReset() {
    return _driver.softReset();
}

/*-----------------------------------------------------------
 * ACCESSORS
 *----------------------------------------------------------*/

bool SCD30::dataAvailable() {
    return _driver.getDataReadyStatus();
}

float SCD30::getCO2() {
    if (co2HasBeenReported == true) // Trigger a new read
        readMeasurement();          // Pull in new co2, humidity, and temp into global vars

    co2HasBeenReported = true;

    return (uint16_t)co2; // Cut off decimal as co2 is 0 to 10,000}
}

float SCD30::getTemperature() {
    if (temperatureHasBeenReported == true) // Trigger a new read
        readMeasurement();                  // Pull in new co2, humidity, and temp into global vars

    temperatureHasBeenReported = true;

    return temperature;
}

float SCD30::getHumidity() {
    if (humidityHasBeenReported == true) // Trigger a new read
        readMeasurement();               // Pull in new co2, humidity, and temp into global vars

    humidityHasBeenReported = true;

    return humidity;
}

uint16_t SCD30::getMeasurementInterval() {
    return _driver.getMeasurementInterval();
}

uint16_t SCD30::getTemperatureOffset() {
    return _driver.getTemperatureOffset();
}

uint8_t SCD30::getAutomaticSelfCalibrationStatus() {
    return _driver.getAutomaticSelfCalibrationStatus();
}

uint16_t SCD30::getForcedCalibrationValue() {
    return _driver.getForcedCalibrationValue();
}

/*-----------------------------------------------------------
 * PRIVATE METHODS
 *----------------------------------------------------------*/

bool SCD30::readMeasurement() {
    Measurement m = _driver.readMeasurement();

    // TODO: add check for Measurement_FAIL

    // Now copy the uint32s into their associated floats
    memcpy(&co2, &m.co2, sizeof(co2));
    memcpy(&temperature, &m.temperature, sizeof(temperature));
    memcpy(&humidity, &m.humidity, sizeof(humidity));

    // Mark our global variables as fresh
    co2HasBeenReported = false;
    humidityHasBeenReported = false;
    temperatureHasBeenReported = false;

    return true; // Success! New data available in globals.
}