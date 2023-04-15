#include "flprog_BMP180.h"

FLProgBMP180::FLProgBMP180(AbstractFLProgI2C *device)
{
    i2cDevice = device;
    addres = 0x77;
}

void FLProgBMP180::readInt(uint8_t addr, int16_t &value)
{
    uint8_t data[2];
    data[0] = addr;
    readBytes(data, 2);
    if (codeError)
    {
        return;
    }
    value = (int16_t)((data[0] << 8) | data[1]);
}

void FLProgBMP180::readUInt(uint8_t address, uint16_t &value)
{
    uint8_t data[2];
    data[0] = address;
    readBytes(data, 2);
    if (codeError)
    {
        return;
    }
    value = (((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

void FLProgBMP180::readBytes(uint8_t *values, uint8_t length)
{
    codeError = i2cDevice->fullWrite(addres, values[0]);
    if (codeError)
    {
        createError();
        return;
    }
    codeError = i2cDevice->fullRead(addres, values, length);
    if (codeError)
    {
        createError();
        return;
    }
}

void FLProgBMP180::initDevice()
{
    double c3, c4, b1;
    readInt(0xAA, AC1);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xAC, AC2);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xAE, AC3);
    if (codeError)
    {
        createError();
        return;
    }
    readUInt(0xB0, AC4);
    if (codeError)
    {
        createError();
        return;
    }
    readUInt(0xB2, AC5);
    if (codeError)
    {
        createError();
        return;
    }
    readUInt(0xB4, AC6);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xB6, VB1);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xB8, VB2);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xBA, MB);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xBC, MC);
    if (codeError)
    {
        createError();
        return;
    }
    readInt(0xBE, MD);
    if (codeError)
    {
        createError();
        return;
    }
    c3 = 160.0 * pow(2, -15) * AC3;
    c4 = pow(10, -3) * pow(2, -15) * AC4;
    b1 = pow(160, 2) * pow(2, -30) * VB1;
    c5 = (pow(2, -15) / 160) * AC5;
    c6 = AC6;
    mc = (pow(2, 11) / pow(160, 2)) * MC;
    md = MD / 160.0;
    x0 = AC1;
    x1 = 160.0 * pow(2, -13) * AC2;
    x2 = pow(160, 2) * pow(2, -25) * VB2;
    y0 = c4 * pow(2, 15);
    y1 = c4 * c3;
    y2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2, -20);
    p2 = 3038.0 * 100.0 * pow(2, -36);
    isInit = true;
}

void FLProgBMP180::createError()
{
    isInit = false;
    gotoStepWithDelay(FLPROG_SENSOR_WAITING_READ_STEP, 500);
}

void FLProgBMP180::pool()
{
    checkReadPeriod();
    checkDelay();
    checkNeededRead();
    if (step == FLPROG_BMP180_READ_TEMPERATURE_STEP)
    {
        readTemperature();
    }
    if (step == FLPROG_BMP180_READ_PRESSURE_STEP)
    {
        readPressure();
    }
}

void FLProgBMP180::readSensor()
{
    if (!isInit)
    {
        initDevice();
    }
    if (codeError)
    {
        createError();
        return;
    }
    startTemperature();
}

void FLProgBMP180::startTemperature()
{
    uint8_t data[2], result;
    data[0] = FLPROG_BMP180_REG_CONTROL;
    data[1] = FLPROG_BMP180_COMMAND_TEMPERATURE;
    codeError = codeError = i2cDevice->fullWrite(addres, data, 2);
    if (codeError)
    {
        createError();
        return;
    }
    gotoStepWithDelay(FLPROG_BMP180_READ_TEMPERATURE_STEP, 5);
}

void FLProgBMP180::readTemperature()
{
    uint8_t data[2];
    double tu, a;
    data[0] = FLPROG_BMP180_REG_RESULT;
    readBytes(data, 2);
    if (codeError)
    {
        createError();
        return;
    }
    tu = (data[0] * 256.0) + data[1];
    a = c5 * (tu - c6);
    temperature = a + (mc / (a + md));
    startPressure();
}

void FLProgBMP180::startPressure()
{
    uint8_t data[2], result, delay;

    data[0] = FLPROG_BMP180_REG_CONTROL;

    switch (oversampling)
    {
    case 0:
        data[1] = FLPROG_BMP180_COMMAND_PRESSURE0;
        delay = 5;
        break;
    case 1:
        data[1] = FLPROG_BMP180_COMMAND_PRESSURE1;
        delay = 8;
        break;
    case 2:
        data[1] = FLPROG_BMP180_COMMAND_PRESSURE2;
        delay = 14;
        break;
    case 3:
        data[1] = FLPROG_BMP180_COMMAND_PRESSURE3;
        delay = 26;
        break;
    default:
        data[1] = FLPROG_BMP180_COMMAND_PRESSURE0;
        delay = 5;
        break;
    }
    codeError = i2cDevice->fullWrite(addres, data, 2);
    if (codeError)
    {
        createError();
        return;
    }
    gotoStepWithDelay(FLPROG_BMP180_READ_PRESSURE_STEP, delay);
}

void FLProgBMP180::readPressure()
{
    uint8_t data[3];
    double pu, s, x, y, z;
    data[0] = FLPROG_BMP180_REG_RESULT;
    readBytes(data, 3);
    if (codeError)
    {
        createError();
        return;
    }
    pu = (data[0] * 256.0) + data[1] + (data[2] / 256.0);
    s = temperature - 25.0;
    x = (x2 * pow(s, 2)) + (x1 * s) + x0;
    y = (y2 * pow(s, 2)) + (y1 * s) + y0;
    z = (pu - x) / y;
    pressure = (p2 * pow(z, 2)) + (p1 * z) + p0;
    step = FLPROG_SENSOR_WAITING_READ_STEP;
}

double FLProgBMP180::sealevel(double P, double A)
{
    return (P / pow(1 - (A / 44330.0), 5.255));
}

double FLProgBMP180::altitude(double P, double P0)
{
    return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}

void FLProgBMP180::setOversampling(uint8_t value)
{
    if (value > 3)
    {
        return;
    }
    oversampling = value;
}