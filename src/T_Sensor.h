#ifndef T_Sensor_H
#define T_Sensor_H

#ifdef __cplusplus

#include <stdint.h>
#include <Arduino.h>        // To use digital/Analog read/write, Delays and processor frequency
#include <pins_arduino.h>
#include "registers.h"
#include "T_Sensor_GPIOs.h"

// you can exclude onewire_search by defining that to 0
// To search the sensor family and select the adress
#ifndef T_Sensor_Search
#define T_Sensor_Search 1
#endif


// enum T_Type
// {
//     DS_18B20,
//     DS_18S20,
//     DS_1822
// };


 class T_sensor
{
    private:
        IO_REG_TYPE bitmask;
        volatile IO_REG_TYPE *baseReg;
/*
============For searching===============
*/
// #if T_Sensor_Search
    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    bool LastDeviceFlag;
// #endif
    public:
    // Constructer to initialize and declare the pin for sensor
        T_sensor(uint8_t pin) {
        pinMode(pin,INPUT);
        bitmask = PIN_TO_BITMASK(pin);
        baseReg = PIN_TO_BASEREG(pin);
        reset_search();
        }


// To rese the cycle if the device does not respond
    uint8_t reset(void);

// To reset the search for new devices
    void reset_search(void);

// To search the adress of the device
    bool search(uint8_t *newAddr, bool search_mode = true);
// To stop the the power to the pin of the sensor
    void depower(void);

// To select the ROM to read the data
    void select(const uint8_t rom[8]);

// To read the data
    uint8_t read(void);

// To read the bit
    uint8_t read_bit(void);

// For the conversion
    void write(uint8_t v, uint8_t pow=0);

// To set the conversion
    void write_bit(uint8_t v);

// To get the temperature in Celcius
    float celcius_read(const uint8_t type, uint8_t address[8]);

// To read the temperature in Fahrenheit
    float fahrenheit_read(const uint8_t type, byte address[8]);
};

// Prevent this name from leaking into Arduino sketches
#ifdef IO_REG_TYPE
#undef IO_REG_TYPE
#endif

#endif  // __cplusplus
#endif  
//T_Sensor_h