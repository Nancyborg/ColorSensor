#include "mbed.h"
#include "ColorSensor.h"

Serial pc(USBTX, USBRX);

int main(void)
{
    pc.baud(115200);
    ColorSensor sensor(I2C(p9, p10), p8);

    pc.printf("Calibration blanc dans 3sec...\r\n");
    wait(3);
    pc.printf("Calibration en cours...\r\n");

    sensor.performWhiteCalibration();

    pc.printf("Calibration terminee\r\n");

    while (true)
    {
        ColorSensor::Color col;

        if (!sensor.readRGB(col))
            pc.printf("erreur de lecture\r\n");

        wait(0.1);
    }
}
