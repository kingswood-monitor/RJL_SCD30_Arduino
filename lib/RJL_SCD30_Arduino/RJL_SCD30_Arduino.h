#include <Arduino.h>

#include <RJL_SCD30_Arduino_Driver.h>

class SCD30 {

    SCD30Driver _driver;

    // Global main datums
    float co2 = 0;
    float temperature = 0;
    float humidity = 0;

    // These track the staleness of the current data
    // This allows us to avoid calling readMeasurement() every time individual datums are requested
    bool co2HasBeenReported = true;
    bool humidityHasBeenReported = true;
    bool temperatureHasBeenReported = true;

    bool readMeasurement();

  public:
    /*-----------------------------------------------------------
     * CREATORS
     *----------------------------------------------------------*/

    SCD30(void);

    /*-----------------------------------------------------------
     * MANIPULATORS
     *----------------------------------------------------------*/

    // Begin and set ambient airpressure to 1013mb
    bool begin(TwoWire &wirePort = Wire);
    // Stop reporting
    bool stop();
    // Offset in degC. +ve offset reduces reported temperature
    bool setTemperatureOffset(float tempOffset);
    // Pressure in millibars
    bool setAmbientPressure(uint16_t pressure);
    // Interval in seconds
    bool setMeasurementInterval(uint16_t interval);
    // Altitude in metres above sea level
    bool setAltitudeCompendation(uint16_t altitude);
    // Value in range (400-2000) ppm. Run for at least 2 mins * 2s sample before applying.
    bool setForcedCalibrationValue(uint16_t cref);

    bool activateAutomaticSelfCalibration();
    bool deactivateAutomaticSelfCalibration();


    bool softReset();

    /*-----------------------------------------------------------
     * ACCESSORS
     *----------------------------------------------------------*/

    bool dataAvailable();

    float getCO2();
    float getTemperature();
    float getHumidity();

    uint16_t getMeasurementInterval();
    uint16_t getTemperatureOffset();
    uint8_t getAutomaticSelfCalibrationStatus();
    uint16_t getForcedCalibrationValue();
};
