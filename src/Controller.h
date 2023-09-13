#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "WS_V2.h"
#include "WIFI.h"
#include <Wire.h>
#include <RTClib.h>
#include <Arduino.h>

#define LOGGER Serial

#define AIR_PWM     7
#define FREQ        5000
#define AIR_PIN     PORT_C2
#define RESOLUTION  8

#define TEMPERATURE_MIN  -50 // Minimum temperature value (in Celsius)
#define TEMPERATURE_MAX  150
#define ADC__RESOLUTION  4095 
#define REFERENCE 3.3

class Controller {
private:
    void setUpI2C();
    void setUpIOS();
    void setUpLogger();
    void setUpAnalogInputs();
    void setUpAnalogOutputs();
    void setUpDigitalInputs();
    void setUpDigitalOutputs();
public:
    ~Controller();
    Controller(/* args */);
    
    void init();
    uint64_t readAnalogInput(uint8_t input);
    bool readDigitalInput(uint8_t input);
    void writeAnalogOutput(uint8_t output, uint8_t value);
    void writeDigitalOutput(uint8_t output, uint8_t value);
    void setUpRTC();
    DateTime getDateTime();
    float readTempFrom(uint8_t channel);
    


};

#endif