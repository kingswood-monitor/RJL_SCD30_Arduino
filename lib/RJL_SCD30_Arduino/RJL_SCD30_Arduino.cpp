#include "RJL_SCD30_Arduino.h"

/*-----------------------------------------------------------
 * CREATORS
 *----------------------------------------------------------*/

SCD30::SCD30() {
}

/*-----------------------------------------------------------
 * MANIPULATORS
 *----------------------------------------------------------*/

bool SCD30::begin(TwoWire &wirePort) {
    
    if (_driver.begin(wirePort) != true)
    {
        return false;
    }

    return true;
}

/*-----------------------------------------------------------
 * ACCESSORS
 *----------------------------------------------------------*/

bool SCD30::dataAvailable() {
    return _driver.getDataReadyStatus();
}

float SCD30::getCO2() {
}
