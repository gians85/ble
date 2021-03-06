#include "Blue1.h"
Serial serialport(USBTX, USBRX);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalIn but1(PUSH1);
DigitalIn but2(PUSH2);
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS); // mosi, miso, sclk

Blue1 blue1(&serialport, &led1, &led2, &led3, &spi);

/*******************************************************************************
* Function Name  : MAIN
* Description    : MAIN FUNCTION.
* Input          : None.
* Return         : None.
******************************************************************************/
int main() {
    /* board init */
    blue1.boardInit();

    /* imu init
    blue1.configIMU();*/

    /* BlueNRG-1 stack init
    blue1.stackInit();*/

    /* Device Init
    blue1.sensorDeviceInit();*/

    /* Device Connectable
    blue1.setDeviceConnectable();*/

    blue1.printSensor();

}