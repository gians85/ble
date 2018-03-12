#ifndef _BLUE1_H_
#define _BLUE1_H_

/*******************************************************************************
*                                 INCLUDES
*******************************************************************************/
#include "mbed.h"
#include "LSM6DS3.h"


/*******************************************************************************
*                                 DEFINES
*******************************************************************************/
#define BLE_SENSOR_VERSION_STRING "1.0.0"
#define UPDATE_CONN_PARAM 0 // Can be set to 1 only when no low power mode is used
#define ADV_INTERVAL_MIN_MS  1000
#define ADV_INTERVAL_MAX_MS  1200
#define SENSOR_EMULATION

/*******************************************************************************
*                                     DEBUG
*******************************************************************************/
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


/*******************************************************************************
*                               CLASS BLUE1
*******************************************************************************/
class Blue1{

private:
    //board
    Serial *serialport;
    DigitalOut *led1;
    DigitalOut *led2;
    DigitalOut *led3;
    DigitalIn *but1;
    DigitalIn *but2;
    SPI *spi; // mosi, miso, sclk
    //imu
    char IMU_register(char, char);
    //blue
    uint8_t ret;
    //stackInit
    //sensorDeviceInit
    uint8_t Sensor_DeviceInit(void);
    //Sensor_DeviceInit
    #define SENSOR_TIMER 1
    static uint16_t acceleration_update_rate;
    static uint8_t sensorTimer_expired;


public:
    Blue1(Serial *, DigitalOut *, DigitalOut *, DigitalOut *, SPI *);
    ~Blue1();
    // board
    void boardInit(void);
    //imu
    void configIMU(void);
    float readTempIMU(void);
    void readAccIMU(int16_t *);
    void readGyroIMU(int16_t *);
    void printSensor(void);
    //blue
    //stackInit
    void stackInit(void);
    //sensorDeviceInit
    void sensorDeviceInit(void);
    //setDeviceConnectable
    void setDeviceConnectable(void);







}; // end class BLue1

#endif //_BLUE1_H_
