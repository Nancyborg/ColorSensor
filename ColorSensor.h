#ifndef _COLORSENSOR_H
#define _COLORSENSOR_H

#include <mbed.h>

class ColorSensor
{
    public:
        struct Color
        {
            int red;
            int green;
            int blue;
            int clear;
        };

        ColorSensor(I2C i2c, PinName ledpin);
        bool readRGB(Color &color);
        void performWhiteCalibration(void);

    private:
        enum Register : unsigned char {
            REG_CTRL = 0x00,
            REG_CONFIG,

            REG_CAP_RED = 0x06,
            REG_CAP_GREEN,
            REG_CAP_BLUE,
            REG_CAP_CLEAR,

            REG_INT_RED_LO = 0x0A,
            REG_INT_RED_HI,
            REG_INT_GREEN_LO,
            REG_INT_GREEN_HI,
            REG_INT_BLUE_LO,
            REG_INT_BLUE_HI,
            REG_INT_CLEAR_LO,
            REG_INT_CLEAR_HI,

            REG_DATA_RED_LO = 0x40,
            REG_DATA_RED_HI,
            REG_DATA_GREEN_LO,
            REG_DATA_GREEN_HI,
            REG_DATA_BLUE_LO,
            REG_DATA_BLUE_HI,
            REG_DATA_CLEAR_LO,
            REG_DATA_CLEAR_HI,

            REG_OFFSET_RED = 0x48,
            REG_OFFSET_GREEN,
            REG_OFFSET_BLUE,
            REG_OFFSET_CLEAR
        };

        int readRegister16(unsigned char addrbase);
        int readRegister(unsigned char addr);
        bool writeRegister(unsigned char addr, unsigned char val);
        bool writeRegister16(unsigned char addrbase, unsigned short val);
        void calibrateClear(void);
        void calibrateColors(void);
        void calibrateCapacitors(void);

        int m_addr;
        I2C m_i2c;
        DigitalOut m_led;
};

#endif
