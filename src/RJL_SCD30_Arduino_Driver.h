#pragma once

#include <Arduino.h>

#include <Wire.h>

#define SDA 21
#define SCL 22

// Structure of the output from readMeasurement()
typedef struct Measurement_t
{
    uint32_t co2;
    uint32_t temperature;
    uint32_t humidity;
} Measurement;

const Measurement Measurement_FAIL = {9999, 999, 999};

// The default I2C address for the SCD30 is 0x61.
#define SCD30_ADDRESS 0x61

// Available commands
#define COMMAND_CONTINUOUS_MEASUREMENT 0x0010      // 1.4.1
#define COMMAND_STOP_CONTINUOUS_MEASUREMENT 0x0104 // 1.4.2
#define COMMAND_SET_MEASUREMENT_INTERVAL 0x4600
#define COMMAND_GET_DATA_READY 0x0202
#define COMMAND_READ_MEASUREMENT 0x0300
#define COMMAND_AUTOMATIC_SELF_CALIBRATION 0x5306
#define COMMAND_SET_FORCED_RECALIBRATION_FACTOR 0x5204
#define COMMAND_SET_TEMPERATURE_OFFSET 0x5403
#define COMMAND_SET_ALTITUDE_COMPENSATION 0x5102
#define COMMAND_GET_FIRMWARE_VERSION 0xD100
#define COMMAND_SOFT_RESET 0xD304

class SCD30Driver {

    TwoWire *_i2cPort; // The generic connection to user's chosen I2C hardware

  public:
    /*-----------------------------------------------------------
     * CREATORS
     *----------------------------------------------------------*/

    bool begin(TwoWire &wirePort);

    /*-----------------------------------------------------------
     * MANIPULATORS
     *----------------------------------------------------------*/

    // Sensor commands

    /**
     * 1.4.1 Trigger continuous measurement with optional ambient pressure compensation
     *
     * Starts continuous measurement of the SCD30 to measure CO2 concentration,
     * humidity and temperature. Measurement data which is not read from the
     * sensor will be overwritten. The measurement interval is adjustable via
     * the command documented in chapter 1.4.3, initial measurement rate is 2s.
     *
     * @param pressureOffset (optional) Pressure in millibars
     * @return true if continuous measurement was triggered
     */
    bool triggerContinuousMeasurement();
    bool triggerContinuousMeasurement(uint16_t pressureOffset);

    /**
     * 1.4.2 Stop continuous measurement
     *
     * Stops the continuous measurement of the SCD30
     * .
     * @return true if continuous measurement was stopped
     */
    bool stopContinuousMeasurement();

    /**
     * 1.4.3 Set measurement interval
     *
     * Sets the interval used by the SCD30 sensor to measure in continuous measurement
     * mode (see chapter 1.4.1). Initial value is 2 s. The chosen measurement interval
     * is saved in non-volatile memory and thus is not reset to its initial value after
     * power up. Velues less than 2 are set to 2. Values more than 1800 are set to 1800.
     *
     * @param intervalSeconds The interval in seconds, range [2..1800]
     * @return true if the measurement interval was set
     */
    bool setMeasurementInterval(uint16_t intervalSeconds);

    /**
     * 1.4.6 (De)Activate Automatic Self Calibration (ASC)
     *
     * Continuous automatic self-calibration can be (de-)activated with the following command.
     * When activated for the first time a period of minimum 7 days is needed so that the
     * algorithm can find its initial parameter set for ASC. The sensor has to be exposed to
     * fresh air for at least 1 hour every day. Also during that period, the sensor may not be
     * disconnected from the power supply, otherwise the procedure to find calibration
     * parameters is aborted and has to be restarted from the beginning. The successfully
     * calculated parameters are stored in non-volatile memory of the SCD30 having the effect
     * that after a restart the previously found parameters for ASC are still present. Note
     * that the most recently found self-calibration parameters will be actively used for
     * selfcalibration disregarding the status of this feature. Finding a new parameter set
     * by the here described method will always overwrite the settings from external
     * recalibration (see chapter 0) and vice-versa. The feature is switched off by default.
     *
     * @return true if ASC was (de)activated
     */
    bool activateAutomaticSelfCalibration();
    bool deactivateAutomaticSelfCalibration();

    /**
     * 1.4.6 Set Forced Recalibration value (FRC)
     *
     * Forced recalibration (FRC) is used to compensate for sensor drifts when a reference value
     * of the CO2 concentration in close proximity to the SCD30 is available. For best results,
     * the sensor has to be run in a stable environment in continuous mode at a measurement rate
     * of 2s for at least two minutes before applying the FRC command and sending the reference
     * value. Setting a reference CO 2 concentration by the method described here will always
     * supersede corrections from the ASC (see chapter 1.4.6) and vice-versa. The reference CO2
     * concentration has to be within the range 400 ppm ≤ cref (CO2 ) ≤ 2000 ppm. The FRC method
     * imposes a permanent update of the CO 2calibration curve which persists after repowering the
     * sensor.
     *
     * @param cref The new reference CO2 concentration (400 ppm ≤ cref (CO2 ) ≤ 2000 ppm)
     * @return true if cref was valid and recalibration was forced
     */
    bool setForcedCalibrationValue(uint16_t cref);

    /**
     * 1.4.7 Set Temperature Offset
     *
     * Set the temperature offset to non-volatile memory
     *
     * @param temp The temperature offset (degress)
     * @return true if the temperature offset was set
     */
    bool setTemperatureOffset(uint16_t temp_offset);

    /**
     * 1.4.8 Set Altitude Compensation
     *
     * Set the altitude compensation. This is ignored if atmospheric pressure is set.
     *
     * @param altitude The reference altitude (metres)
     * @return true if altitude was set
     */
    bool setAltitudeCompensation(uint16_t temp_offset);

    /**
     * 1.4.10 Soft Reset
     *
     * Soft reset
     *
     * @return true if reset
     */
    bool softReset();

    // Low level commands
    bool sendCommand(uint16_t command);
    bool sendCommand(uint16_t command, uint16_t arguments);

    /*-----------------------------------------------------------
     * ACCESSORS
     *----------------------------------------------------------*/

    /**
     * 1.4.3 Get measurement interval
     *
     * @return The measurement interval (seconds)
     */
    uint16_t getMeasurementInterval();

    /**
     * 1.4.4 Get data ready status
     *
     * Data ready command is used to determine if a measurement can be read from the
     * sensor’s buffer. Whenever there is a measurement available from the internal buffer
     * this command returns 1 and 0 otherwise. As soon as the measurement has been read
     * the return value changes to 0.
     *
     * @return true when a measurement is available to be read by the sensor
     */
    bool getDataReadyStatus();

    /**
     * 1.4.5 Read measurement
     *
     * When new measurement data is available it can be read out with the following command
     *
     * @return TBA
     */
    Measurement readMeasurement();

    /**
     * 1.4.6 Get Automatic Self Calibration status (ASC)
     *
     * Gets the current
     * @return 1 if activated, 0 if deactivated
     */
    uint8_t getAutomaticSelfCalibrationStatus();

    /**
     * 1.4.6 Get Forced Recalibration value (FRC)
     *
     * @return The current recalibration value, or -1 as sentinal for failure
     */
    uint16_t getForcedCalibrationValue();

    /**
     * 1.4.7 Get Temperature Offset
     *
     * @return The current temperature offset, or 999 as sentinal for failure
     */
    uint16_t getTemperatureOffset();

    /**
     * 1.4.8 Get Altitude Compensation
     *
     * @return The current altitude compensation, or -1 as sentinal for failure
     */
    uint16_t getAltitudeCompensation();

    /**
     * 1.4.9 Read firmware versionn
     *
     * @return The current firmware verions
     */
    uint16_t getFirmwareVersion();

    // Sensor commands

    uint16_t readRegister(uint16_t registerAddress);

    /*-----------------------------------------------------------
     * HELPERS
     *----------------------------------------------------------*/
    uint8_t computeCRC8(uint8_t data[], uint8_t len);
    void writeUint16(uint16_t byte);
    void writeUint16WithCRC(uint16_t byte);
};