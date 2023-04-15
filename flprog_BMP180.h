#pragma once
#include "Arduino.h"
#include "flprogUtilites.h"
#include "flprogI2C.h"
#include <stdio.h>
#include <math.h>

#define FLPROG_BMP180_REG_CONTROL 0xF4
#define FLPROG_BMP180_REG_RESULT 0xF6

#define FLPROG_BMP180_COMMAND_TEMPERATURE 0x2E
#define FLPROG_BMP180_COMMAND_PRESSURE0 0x34
#define FLPROG_BMP180_COMMAND_PRESSURE1 0x74
#define FLPROG_BMP180_COMMAND_PRESSURE2 0xB4
#define FLPROG_BMP180_COMMAND_PRESSURE3 0xF4

#define FLPROG_BMP180_READ_TEMPERATURE_STEP 10
#define FLPROG_BMP180_READ_PRESSURE_STEP 11

class FLProgBMP180 : public FLProgI2cStepWorkSensor
{
public:
    FLProgBMP180(AbstractFLProgI2C *device);
    void pool();
    double getTemperature() { return temperature; };
    double getPressure() { return pressure; };
    double sealevel(double P, double A);
    double altitude(double P, double P0);
    void setOversampling(uint8_t value);

protected:
    void readInt(uint8_t addr, int16_t &value);
    void readUInt(uint8_t addr, uint16_t &value);
    void readBytes(uint8_t *values, uint8_t length);
    void initDevice();
    void createError();
    virtual void readSensor();
    void startTemperature();
    void readTemperature();
    void startPressure();
    void readPressure();
    bool isInit = false;
    int16_t AC1, AC2, AC3, VB1, VB2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;
    double c5, c6, mc, md, x0, x1, x2, y0, y1, y2, p0, p1, p2;
    double temperature;
    double pressure;
    uint8_t oversampling = 3;
};