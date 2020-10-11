#include <Arduino.h>
#include <unity.h>

#include <Wire.h>

#include <RJL_SCD30_Arduino_Driver.h>

SCD30Driver driver;

uint16_t old_measurement_interval = 2;
uint16_t old_forced_calibration_Value = 408;
uint16_t old_temperature_offset = 0;
uint16_t old_altitude_compensation = 0;

void setUp(void) {
    driver.begin(Wire);
    Wire.begin(SDA, SCL);
    // Capture the state of the sensor
    old_measurement_interval = driver.getMeasurementInterval();
    old_forced_calibration_Value = driver.getForcedCalibrationValue();
    old_temperature_offset = driver.getTemperatureOffset();
    old_altitude_compensation = driver.getAltitudeCompensation();
}

void tearDown(void) {
    // Return SCD30 to a known state
    driver.triggerContinuousMeasurement();
    driver.setMeasurementInterval(old_measurement_interval);
    driver.setForcedCalibrationValue(old_forced_calibration_Value);
    driver.setTemperatureOffset(old_temperature_offset);
    driver.setAltitudeCompensation(old_altitude_compensation);
}

// CRC

void test_compute_crc_zero() {
    uint8_t data[2];
    data[0] = 0;
    data[1] = 0;

    TEST_ASSERT_EQUAL_HEX8(0x81, driver.computeCRC8(data, 2));
}

void test_compute_crc_BEEF() {
    uint8_t data[2];
    data[0] = 0xBE;
    data[1] = 0xEF;

    TEST_ASSERT_EQUAL_HEX8(0x92, driver.computeCRC8(data, 2));
}

// 1.4.1 Trigger continuous measurement

void test_trigger_continuous_measurement_no_parameter() {
    TEST_ASSERT_TRUE(driver.triggerContinuousMeasurement());
}
void test_trigger_continuous_measurement_zero() {
    TEST_ASSERT_TRUE(driver.triggerContinuousMeasurement(0));
}
void test_trigger_continuous_measurement_non_zero() {
    TEST_ASSERT_TRUE(driver.triggerContinuousMeasurement(1000));
}

// 1.4.2 Stop continuous measurement

void test_stop_continuous_measurement() {
    TEST_ASSERT_TRUE(driver.stopContinuousMeasurement());
}

// 1.4.3 Set measurement interval

void test_set_measurement_interval_too_high() {
    TEST_ASSERT_TRUE(driver.setMeasurementInterval(2500));
}
void test_set_measurement_interval_too_low() {
    TEST_ASSERT_TRUE(driver.setMeasurementInterval(0));
}
void test_set_measurement_interval_valid() {
    TEST_ASSERT_TRUE(driver.setMeasurementInterval(2));
}

// 1.4.3 Get measurement interval

void test_get_measurement_interval() {
    uint16_t interval = driver.getMeasurementInterval();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16_MESSAGE(2, interval, "Measurement interval less than 2 seconds");
}

// 1.4.4 Get data ready status

void test_get_data_ready_status() {
    driver.triggerContinuousMeasurement();
    while (!driver.getDataReadyStatus())
    {
        TEST_MESSAGE("Waiting for ready status...");
    }
    TEST_ASSERT_TRUE(driver.getDataReadyStatus());
}

// 1.4.5 Read measurement

void test_read_measurement() {
    Measurement m = driver.readMeasurement();
    bool bDidNotFail = !((m.co2 == 9999) || (m.temperature == 999) || (m.humidity == 999));
    TEST_ASSERT_TRUE_MESSAGE(bDidNotFail, "FAIL measurement received");

    float co2 = 0;
    memcpy(&co2, &m.co2, sizeof(co2));
    bool bCO2IsValid = ((co2 >= 0) && (co2 <= 40000));
    TEST_ASSERT_TRUE_MESSAGE(bCO2IsValid, "Invalid CO2 reading");

    float humidity = 0;
    memcpy(&humidity, &m.humidity, sizeof(humidity));
    bool bHumidityIsValid = ((humidity >= 0) && (humidity <= 100));
    TEST_ASSERT_TRUE_MESSAGE(bHumidityIsValid, "Invalid Humidity reading");

    float temperature = 0;
    memcpy(&temperature, &m.temperature, sizeof(temperature));
    bool bTemperatureIsValid = ((temperature >= -40) && (temperature <= 70));
    TEST_ASSERT_TRUE_MESSAGE(bTemperatureIsValid, "Invalid Temperature reading");
}

// 1.4.6 (De)Activate Automatic Self Calibration (ASC)
void test_activate_automatic_self_calibration() {
    TEST_ASSERT_TRUE(driver.activateAutomaticSelfCalibration());
}

// 1.4.6 (De)Activate Automatic Self Calibration (ASC)
void test_get_automatic_self_calibration_status() {
    uint8_t status = driver.getAutomaticSelfCalibrationStatus();
    bool bIsValid = ((status == 0) || (status == 1));
    TEST_ASSERT_TRUE_MESSAGE(bIsValid, "Invalid status");
}

// 1.4.6 Set Forced Recalibration value (FRC)
void test_set_forced_recalibration_value() {
    TEST_ASSERT_TRUE(driver.setForcedCalibrationValue(1000));
}

// 1.4.6 Get Forced Recalibration value (FRC)
void test_get_forced_recalibration_value() {
    uint16_t value = driver.getForcedCalibrationValue();
    TEST_ASSERT_LESS_OR_EQUAL_UINT16_MESSAGE(40000, value, "Invalid forced calibration value");
}

// 1.4.7 Set Temperature Offset
void test_set_temperature_offset() {
    TEST_ASSERT_TRUE(driver.setTemperatureOffset(3));
}

// 1.4.7 Get Temperature Offset
void test_get_temperature_offset() {
    uint16_t offset = driver.getTemperatureOffset();
    TEST_ASSERT_TRUE(driver.getTemperatureOffset());
}

// 1.4.8 Set Altitude Compensation
void test_set_altitude_compensation() {
    TEST_ASSERT_TRUE(driver.setAltitudeCompensation(3));
}

// 1.4.8 Get Altitude Compensation
void test_get_altitude_compensation() {
    uint16_t altitude = driver.getAltitudeCompensation();
    TEST_ASSERT_TRUE(driver.getAltitudeCompensation());
}

// 1.4.9 Read firmware versionn
void test_get_firmware_version() {
    TEST_ASSERT_TRUE(driver.getFirmwareVersion());
}

// 1.4.10 Soft reset
void test_soft_reset() {
    TEST_ASSERT_TRUE(driver.softReset());
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_compute_crc_zero);
    RUN_TEST(test_compute_crc_BEEF);

    RUN_TEST(test_trigger_continuous_measurement_no_parameter);
    RUN_TEST(test_trigger_continuous_measurement_zero);
    RUN_TEST(test_trigger_continuous_measurement_non_zero);

    RUN_TEST(test_stop_continuous_measurement);

    RUN_TEST(test_set_measurement_interval_too_high);
    RUN_TEST(test_set_measurement_interval_too_low);
    RUN_TEST(test_set_measurement_interval_valid);
    RUN_TEST(test_get_measurement_interval);

    RUN_TEST(test_get_data_ready_status);

    RUN_TEST(test_read_measurement);

    RUN_TEST(test_activate_automatic_self_calibration);
    RUN_TEST(test_get_automatic_self_calibration_status);

    RUN_TEST(test_set_forced_recalibration_value);
    RUN_TEST(test_get_forced_recalibration_value);

    RUN_TEST(test_set_temperature_offset);
    RUN_TEST(test_get_temperature_offset);

    RUN_TEST(test_set_altitude_compensation);
    RUN_TEST(test_get_altitude_compensation);
    
    RUN_TEST(test_get_firmware_version);

    RUN_TEST(test_soft_reset);

    UNITY_END();
}

void loop() {
}