#include <Arduino.h>
#include <unity.h>

#include <Wire.h>

#include <RJL_SCD30_Arduino.h>

SCD30 airSensor;

uint16_t old_measurement_interval = 0;

void setUp(void) {
    airSensor.begin();
    old_measurement_interval = airSensor.getMeasurementInterval();
}

void tearDown(void) {
    airSensor.setMeasurementInterval(old_measurement_interval);
}

/*-----------------------------------------------------------
 * MANIPULATORS
 *----------------------------------------------------------*/

void test_begin(void) {
    TEST_ASSERT_TRUE(airSensor.begin() == true);
}

void test_stop() {
    TEST_ASSERT_TRUE(airSensor.stop() == true);
}

void test_set_ambient_pressure() {
    TEST_ASSERT_TRUE(airSensor.setAmbientPressure(1000) == true);
}

void test_set_measurement_interval() {
    TEST_ASSERT_TRUE(airSensor.setMeasurementInterval(5));
}

/*-----------------------------------------------------------
 * ACCESSORS
 *----------------------------------------------------------*/

void test_data_available() {
    TEST_ASSERT_TRUE(airSensor.dataAvailable() == true);
}

void test_get_measurement_interval() {
    uint16_t interval = airSensor.getMeasurementInterval();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(2, interval);
}

void test_get_readings_returns_valid() {
    while (!airSensor.dataAvailable())
    {
        delay(10);
    }
    float co2 = airSensor.getCO2();
    float temperature = airSensor.getTemperature();
    float humidity = airSensor.getHumidity();

    // bool bCO2IsValid = ((co2 >= 0) && (co2 <= 40000));
    // bool bHumidityIsValid = ((humidity >= 0) && (humidity <= 100));
    // bool bTemperatureIsValid = ((temperature >= -40) && (temperature <= 70));

    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(400, co2, "CO2 less than 400ppm");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(40000, co2, "CO2 greater than 40,000 ppm");

    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(0, humidity, "Humidity less than 0%");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(100, humidity, "Humidity greater than 100%");

    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(-40, temperature, "Temperature less than -40deg");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(70, temperature, "Temperature greater than 70deg");
}

// END

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_begin);
    RUN_TEST(test_stop);

    RUN_TEST(test_data_available);

    RUN_TEST(test_set_ambient_pressure);

    RUN_TEST(test_set_measurement_interval);
    RUN_TEST(test_get_measurement_interval);

    RUN_TEST(test_get_readings_returns_valid);

    UNITY_END();
}

void loop() {
}