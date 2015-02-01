#include "ColorSensor.h"
#include <algorithm>

extern Serial pc;

ColorSensor::ColorSensor(I2C i2c, PinName ledpin)
    : m_addr(0xE8), m_i2c(i2c), m_led(ledpin)
{
    // TODO : calculer les valeurs optimales

/*
    writeRegister(REG_CAP_RED, 0x05);
    writeRegister(REG_CAP_GREEN, 0x05);
    writeRegister(REG_CAP_BLUE, 0x05);
    writeRegister(REG_CAP_CLEAR, 0x05);

    writeRegister(REG_INT_RED_LO, 0xC4);
    writeRegister(REG_INT_RED_HI, 0x09);
    writeRegister(REG_INT_GREEN_LO, 0xC4);
    writeRegister(REG_INT_GREEN_HI, 0x09);
    writeRegister(REG_INT_BLUE_LO, 0xC4);
    writeRegister(REG_INT_BLUE_HI, 0x09);
    writeRegister(REG_INT_CLEAR_LO, 0xC4);
    writeRegister(REG_INT_CLEAR_HI, 0x09);
*/
    writeRegister(REG_CONFIG, 1); // Trim offset
}

int ColorSensor::readRegister(unsigned char reg)
{
    char val = 0xFF;

    if (m_i2c.write(m_addr, (char*) &reg, 1, true) != 0)
        return -1;

    if (m_i2c.read(m_addr + 1, &val, 1) != 0)
        return -1;

    return val;
}

int ColorSensor::readRegister16(unsigned char regbase)
{
    int low = readRegister(regbase);
    int high = readRegister(regbase + 1);

    if (low == -1 || high == -1)
        return -1;

    return (high << 8) | low;
}

bool ColorSensor::writeRegister(unsigned char reg, unsigned char value)
{
    char data[2] = {reg, value};
    return m_i2c.write(m_addr, data, 2) == 0;
}

bool ColorSensor::writeRegister16(unsigned char regbase, unsigned short value)
{
    if (!writeRegister(regbase, value & 0xFF))
        return false;

    if (!writeRegister(regbase + 1, value >> 8))
        return false;

    return true;
}

bool ColorSensor::readRGB(Color &out)
{
    if (!writeRegister(REG_CTRL, 2))
        return false;

    while (readRegister(REG_CTRL))
        wait_ms(1);

    m_led = 1;

    if (!writeRegister(REG_CTRL, 1))
        return false;

    while (readRegister(REG_CTRL))
        wait_ms(1);

    m_led = 0;

    int red = readRegister16(REG_DATA_RED_LO);
    int green = readRegister16(REG_DATA_GREEN_LO);
    int blue = readRegister16(REG_DATA_BLUE_LO);
    int clear = readRegister16(REG_DATA_BLUE_LO);

    if (red == -1 || green == -1 || blue == -1 || clear == -1)
        return false;

    out.red = red;
    out.green = green;
    out.blue = blue;
    out.clear = clear;

    pc.printf("r = %d  g = %d  b = %d c = %d\r\n", out.red, out.green, out.blue, out.clear);

    int offred = readRegister(REG_OFFSET_RED);
    int offgreen = readRegister(REG_OFFSET_GREEN);
    int offblue = readRegister(REG_OFFSET_BLUE);
    int offclear = readRegister(REG_OFFSET_CLEAR);

    pc.printf("offsets = %d %d %d %d\r\n", offred, offgreen, offblue, offclear);
}

// Fortement inspiré par http://bildr.org/2012/01/adjd-s311_arduino/

void ColorSensor::performWhiteCalibration(void)
{
    calibrateClear();
    calibrateColors();
    calibrateCapacitors();
}

void ColorSensor::calibrateClear(void)
{
    bool gainFound = false;
    int upperBound = 4096;
    int lowerBound = 0;
    int half;

    while (!gainFound)
    {
        half = ((upperBound - lowerBound) / 2) + lowerBound;

        if (half == lowerBound)
            gainFound = true;

        else
        {
            Color c;

            writeRegister16(REG_INT_CLEAR_LO, half);

            readRGB(c);

            if (c.clear > 1000)
                upperBound = half;
            else if (c.clear < 1000)
                lowerBound = half;
            else
                gainFound = true;
        }
    }

    pc.printf("Calibration clear = %d\r\n", half);
}

void ColorSensor::calibrateColors(void)
{
    bool gainFound = false;
    int upperBound = 4096;
    int lowerBound = 0;
    int half;

    while (!gainFound)
    {
        half = ((upperBound - lowerBound) / 2) + lowerBound;

        if (half == lowerBound)
            gainFound = true;

        else
        {
            int minValue = 0;
            Color c;

            writeRegister16(REG_INT_RED_LO, half);
            writeRegister16(REG_INT_GREEN_LO, half);
            writeRegister16(REG_INT_BLUE_LO, half);

            readRGB(c);

            minValue = min(minValue, c.red);
            minValue = min(minValue, c.green);
            minValue = min(minValue, c.blue);

            if (minValue > 1000)
                upperBound = half;
            else if (minValue < 1000)
                lowerBound = half;
            else
                gainFound = true;
        }
    }

    pc.printf("Calibration couleurs = %d\r\n", half);
}

void ColorSensor::calibrateCapacitors(void)
{
    int calibrationRed = 0;
    int calibrationGreen = 0;
    int calibrationBlue = 0;
    bool calibrated = false;

    while (!calibrated)
    {
        Color c;

        writeRegister(REG_CAP_RED, calibrationRed);
        writeRegister(REG_CAP_GREEN, calibrationGreen);
        writeRegister(REG_CAP_BLUE, calibrationBlue);

        readRGB(c);

        if (c.red > 1000 && calibrationRed < 15)
            calibrationRed++;
        else if (c.green > 1000 && calibrationGreen < 15)
            calibrationGreen++;
        else if (c.blue > 1000 && calibrationBlue < 15)
            calibrationBlue++;
        else
            calibrated = true;;
    }

    pc.printf("Capacitor calibration : %d %d %d\r\n", calibrationRed, calibrationGreen, calibrationBlue);
}
