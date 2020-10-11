#include <RJL_SCD30_Arduino_Driver.h>

/*-----------------------------------------------------------
 * MANIPULATORS
 *----------------------------------------------------------*/

bool SCD30Driver::begin(TwoWire &wirePort) {

    _i2cPort = &wirePort;
    _i2cPort->begin(SDA, SCL);

    if (triggerContinuousMeasurement(0) != true)
    {
        return false;
    }
}

// 1.4.1 Trigger continuous measurement
bool SCD30Driver::triggerContinuousMeasurement() {
    triggerContinuousMeasurement(0);
}

bool SCD30Driver::triggerContinuousMeasurement(uint16_t pressureOffset) {
    return (sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressureOffset));
}

// 1.4.2 Stop continuous measurement
bool SCD30Driver::stopContinuousMeasurement() {
    return (sendCommand(COMMAND_STOP_CONTINUOUS_MEASUREMENT));
}

// 1.4.3 Set measurement interval
bool SCD30Driver::setMeasurementInterval(uint16_t intervalSeconds) {
    if (intervalSeconds < 2)
    {
        intervalSeconds = 2;
    }
    if (intervalSeconds > 1800)
    {
        intervalSeconds = 1800;
    }
    return (sendCommand(COMMAND_SET_MEASUREMENT_INTERVAL, intervalSeconds));
}

// 1.4.6 (De)Activate Automatic Self Calibration (ASC)
bool SCD30Driver::toggleActivateAutomaticSelfCalibration() {
    return (sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION));
}

// 1.4.6 Set Forced Recalibration value (FRC)
bool SCD30Driver::setForcedCalibrationValue(uint16_t cref) {
    return (sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, cref));
}

// 1.4.7 Set Temperature Offset
bool SCD30Driver::setTemperatureOffset(uint16_t cref) {
    return (sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, cref));
}

// 1.4.8 Set Altitude Compensation
bool SCD30Driver::setAltitudeCompensation(uint16_t cref) {
    return (sendCommand(COMMAND_SET_ALTITUDE_COMPENSATION, cref));
}

// 1.4.10 Soft reset
bool SCD30Driver::softReset() {
    return (sendCommand(COMMAND_SOFT_RESET));
}

/*-----------------------------------------------------------
 * ACCESSORS
 *----------------------------------------------------------*/

// 1.4.3 Get measurement interval
uint16_t SCD30Driver::getMeasurementInterval() {
    uint16_t response = readRegister(COMMAND_SET_MEASUREMENT_INTERVAL);

    return response;
}

// 1.4.4 Get data ready status
bool SCD30Driver::getDataReadyStatus() {
    uint16_t response = readRegister(COMMAND_GET_DATA_READY);

    if (response == 1)
        return (true);
    return (false);
}

// 1.4.5 Read measurement
Measurement SCD30Driver::readMeasurement() {

    uint32_t tempCO2 = 0;
    uint32_t tempHumidity = 0;
    uint32_t tempTemperature = 0;

    // TODO: Add a repeat/timeout feature
    if (!getDataReadyStatus())
    {
        return Measurement_FAIL;
    }

    sendCommand(COMMAND_READ_MEASUREMENT);

    // Read 18 bytes, return FAIL on I2C error
    const uint8_t receivedBytes = _i2cPort->requestFrom((uint8_t)SCD30_ADDRESS, (uint8_t)18);
    bool error = false;

    if (!_i2cPort->available())
    {
        return Measurement_FAIL;
    }

    // Pack into Measurement
    byte bytesToCrc[2];
    for (byte x = 0; x < 18; x++)
    {
        byte incoming = _i2cPort->read();

        switch (x)
        {
        case 0:
        case 1:
        case 3:
        case 4:
            tempCO2 <<= 8;
            tempCO2 |= incoming;
            bytesToCrc[x % 3] = incoming;
            break;
        case 6:
        case 7:
        case 9:
        case 10:
            tempTemperature <<= 8;
            tempTemperature |= incoming;
            bytesToCrc[x % 3] = incoming;
            break;
        case 12:
        case 13:
        case 15:
        case 16:
            tempHumidity <<= 8;
            tempHumidity |= incoming;
            bytesToCrc[x % 3] = incoming;
            break;
        default:
            // Validate CRC
            const uint8_t foundCrc = computeCRC8(bytesToCrc, 2);
            if (foundCrc != incoming)
            {
                // Serial.printf("Found CRC in byte %u, expected %u, got %u\n", x, incoming, foundCrc);
                error = true;
            }
            break;
        }
    }

    if (error)
    {
        return Measurement_FAIL;
    }

    return Measurement{tempCO2, tempTemperature, tempHumidity};
}

// 1.4.6 Get Automatic Self Calibration status (ASC)
uint8_t SCD30Driver::getAutomaticSelfCalibrationStatus() {

    uint16_t response = readRegister(COMMAND_AUTOMATIC_SELF_CALIBRATION);
    if ((response != 1) && (response != 0))
    {
        return 99; // fail sentinal value;
    }

    return response;
}

// 1.4.6 Get Forced Recalibration value (FRC)
uint16_t SCD30Driver::getForcedCalibrationValue() {
    uint16_t response = readRegister(COMMAND_SET_FORCED_RECALIBRATION_FACTOR);

    return response;
}

// 1.4.7 Get Temperature Offset
uint16_t SCD30Driver::getTemperatureOffset() {
    uint16_t response = readRegister(COMMAND_SET_TEMPERATURE_OFFSET);

    return response;
}

// 1.4.8 Get Altitude Compensation
uint16_t SCD30Driver::getAltitudeCompensation() {
    uint16_t response = readRegister(COMMAND_SET_ALTITUDE_COMPENSATION);

    return response;
}

// 1.4.9 Read firmware version
uint16_t SCD30Driver::getFirmwareVersion() {
    uint16_t response = readRegister(COMMAND_GET_FIRMWARE_VERSION);

    return response;
}

/*-----------------------------------------------------------
 * PRIVATE METHODS - LOW LEVEL METHODS
 *----------------------------------------------------------*/

// Sends just a command, no arguments, no CRC
bool SCD30Driver::sendCommand(uint16_t command) {
    _i2cPort->beginTransmission(SCD30_ADDRESS);
    writeUint16(command);
    if (_i2cPort->endTransmission() != 0)
        return (false); // Sensor did not ACK

    return (true);
}

// Sends a command along with arguments and CRC
bool SCD30Driver::sendCommand(uint16_t command, uint16_t arguments) {
    _i2cPort->beginTransmission(SCD30_ADDRESS);
    writeUint16(command);
    writeUint16WithCRC(arguments);

    if (_i2cPort->endTransmission() != 0)
        return (false); // Sensor did not ACK

    return (true);
}

// Gets two bytes from SCD30
uint16_t SCD30Driver::readRegister(uint16_t registerAddress) {

    if (!sendCommand(registerAddress))
    {
        return 0; // Sensor did not ACK
    }

    delay(3); // See 1.4.1 note on delay between command and read

    _i2cPort->requestFrom((uint8_t)SCD30_ADDRESS, (uint8_t)2);
    if (_i2cPort->available())
    {
        uint8_t msb = _i2cPort->read();
        uint8_t lsb = _i2cPort->read();
        return ((uint16_t)msb << 8 | lsb);
    }
    return (0); // Sensor did not respond
}

void SCD30Driver::writeUint16(uint16_t byte) {
    _i2cPort->write(byte >> 8);   // MSB
    _i2cPort->write(byte & 0xFF); // LSB
}

void SCD30Driver::writeUint16WithCRC(uint16_t byte) {

    uint8_t data[2];
    data[0] = byte >> 8;
    data[1] = byte & 0xFF;
    uint8_t crc = computeCRC8(data, 2); // Calc CRC on the arguments only, not the command

    writeUint16(byte);
    _i2cPort->write(crc);
}

uint8_t SCD30Driver::computeCRC8(uint8_t data[], uint8_t len) {
    uint8_t crc = 0xFF; // Init with 0xFF

    for (uint8_t x = 0; x < len; x++)
    {
        crc ^= data[x]; // XOR-in the next input byte

        for (uint8_t i = 0; i < 8; i++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }

    return crc; // No output reflection
}
