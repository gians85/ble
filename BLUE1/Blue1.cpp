#include "Blue1.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include "bluenrg1_api.h"
#include "bluenrg1_events.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "clock.h"
#include "BlueNRG1_sleep.h"
#include "SensorDemo_config.h"
#include "sensor.h"
#include "gatt_db.h"
#ifdef __cplusplus
}
#endif

uint8_t Services_Max_Attribute_Records[NUMBER_OF_APPLICATION_SERVICES] = {MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2};

/******************************************************************************
* Static members definitions
******************************************************************************/
//Sensor_DeviceInit
uint16_t Blue1::acceleration_update_rate = 200;
uint8_t  Blue1::sensorTimer_expired = FALSE;

/******************************************************************************
* Function Name  : Blue1
* Description    : Constructor
* Input          :
* Return         :
******************************************************************************/
Blue1::Blue1(Serial *serialPtr, DigitalOut *out1Ptr, DigitalOut *out2Ptr, DigitalOut *out3Ptr, SPI *spiPtr){
    //board
    serialport = serialPtr;
    led1 = out1Ptr;
    led2 = out2Ptr;
    led3 = out3Ptr;
    spi  = spiPtr;
}

/******************************************************************************
* Function Name  : ~Blue1
* Description    : Distructor
* Input          :
* Return         :
******************************************************************************/
Blue1::~Blue1(){}


/******************************************************************************
* Function Name  : boardInit
* Description    : Init the board.
* Input          : None.
* Return         : None.
******************************************************************************/
void Blue1::boardInit(){
	// SPI Interface
	spi->format(8, 0);
	spi->frequency(100000);
    // serial interrupt
    //serialport->attach(&Rx_interrupt, Serial::RxIrq);
	// Ready
	*led2 = 1;
    PRINTF("\014Blue1\r\n");
}

/******************************************************************************
* Function Name  : configIMU
* Description    : config inertial sensor
* Input          :
* Return         :
******************************************************************************/
void Blue1::configIMU(){
	IMU_register(0x11, 0x12);
	IMU_register(0x19, 0x38);
	IMU_register(0x10, 0x10);
	IMU_register(0x18, 0x38);
}

/******************************************************************************
* Function Name  : IMU_register
* Description    : access to imu register
* Input          : register name, data to write
* Return         : read data
******************************************************************************/
char Blue1::IMU_register(char reg_name, char data){
	char buf[2];
	if (data == READ){
		reg_name |= READ_1B;
		spi->write(&reg_name, 1, &data, 1);
	}
	else{
		buf[0] = reg_name;
		buf[1] = data;
		spi->write(buf, 2, &data, 0);
	}
	return data;
}

/******************************************************************************
* Function Name  : readTempIMU
* Description    : read IMU temperature
* Input          : none
* Return         : temperature in °C
******************************************************************************/
float Blue1::readTempIMU(){
	int16_t temp = 0;
	temp  = Blue1::IMU_register(OUT_TEMP_H, READ);
	temp <<= 8;
	temp |= Blue1::IMU_register(OUT_TEMP_L, READ);
	return (25.0+(((float)temp)/16));
}

/******************************************************************************
* Function Name  : readAccIMU
* Description    : read IMU temperature
* Input          : none
* Return         : temperature in °C
******************************************************************************/
void Blue1::readAccIMU(int16_t * buf){
	buf[0]  = Blue1::IMU_register(OUT_X_H_A, READ);
	buf[0] <<= 8;
	buf[0] |= Blue1::IMU_register(OUT_X_L_A, READ);
	buf[1]  = Blue1::IMU_register(OUT_Y_H_A, READ);
	buf[1] <<= 8;
	buf[1] |= Blue1::IMU_register(OUT_Y_L_A, READ);
	buf[2]  = Blue1::IMU_register(OUT_Z_H_A, READ);
	buf[2] <<= 8;
	buf[2] |= Blue1::IMU_register(OUT_Z_L_A, READ);
}

/******************************************************************************
* Function Name  : readGyroIMU
* Description    : read IMU temperature
* Input          : none
* Return         : temperature in °C
******************************************************************************/
void Blue1::readGyroIMU(int16_t * buf){
	buf[0]  = Blue1::IMU_register(OUT_X_H_G, READ);
	buf[0] <<= 8;
	buf[0] |= Blue1::IMU_register(OUT_X_L_G, READ);
	buf[1]  = Blue1::IMU_register(OUT_Y_H_G, READ);
	buf[1] <<= 8;
	buf[1] |= Blue1::IMU_register(OUT_Y_L_G, READ);
	buf[2]  = Blue1::IMU_register(OUT_Z_H_G, READ);
	buf[2] <<= 8;
	buf[2] |= Blue1::IMU_register(OUT_Z_L_G, READ);
}

/******************************************************************************
* Function Name  : printSensor
* Description    : Print sensor values on serial port.
* Input          : None.
* Return         : None.
******************************************************************************/
void Blue1::printSensor(){
    int16_t buf[3];
    float a = 0.000061, g = 0.004375;
    // acc_sensor
    Blue1::readAccIMU(buf);
	PRINTF(" ACC(xyz): %.4f  %.4f  %.4f\r\n", buf[0]*a, buf[1]*a, buf[2]*a);
    // gyro_sensor
    Blue1::readGyroIMU(buf);
	PRINTF("GYRO(xyz): %.4f  %.4f  %.4f\r\n", buf[0]*g, buf[1]*g, buf[2]*g);
    // temp_sensor
    PRINTF("TEMP [°C]: %.1f \r\n", Blue1::readTempIMU());
}

/******************************************************************************
* Function Name  : stackInit
* Description    : BlueNRG Stack Initialization.
* Input          : None.
* Return         : None.
******************************************************************************/
void Blue1::stackInit(){
    ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
        while(1);
    }
}

/******************************************************************************
* Function Name  : sensorDeviceInit()
* Description    : Call Sensor_DeviceInit() and manage its ret value.
* Input          : None.
* Return         : None.
******************************************************************************/
void Blue1::sensorDeviceInit(){
    ret = Blue1::Sensor_DeviceInit();
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error in Device Initialization() 0x%02x\r\n", ret);
        while(1);
    }else{
        PRINTF("BLE_STATUS_SUCCESS\r\n");
    }
}

/******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : Status.
 *****************************************************************************/
uint8_t Blue1::Sensor_DeviceInit(){
    PRINTF("Sensor_DeviceInit()\r\n");
    uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
    uint8_t device_name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G'};

    /* Set the device public address */
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
    if(ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* Set the TX power -2 dBm */
    aci_hal_set_tx_power_level(1, 4);

    /* GATT Init */
    ret = aci_gatt_init();
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gatt_init() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* GAP Init */
    ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_init() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* Update device name */
    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name), device_name);
    if(ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gatt_update_char_value() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x (new API prototype) */
    ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7,
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456,
                                               0x00);
    if(ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_authentication_requirement()failed: 0x%02x\r\n", ret);
        return ret;
    }

    PRINTF("BLE Stack Initialized with SUCCESS\r\n");

    /* Add services and Characteristics */
/*
    #ifndef SENSOR_EMULATION // User Real sensors /
    #warning SENSOR_EMULATION
    Init_Accelerometer();
    Init_Pressure_Temperature_Sensor();
    #endif
    // Add ACC service and Characteristics
    ret = Add_Acc_Service();
    if(ret == BLE_STATUS_SUCCESS) {
        PRINTF("Acceleration service added successfully.\r\n");
    } else {
        PRINTF("Error while adding Acceleration service: 0x%02x\r\n", ret);
        return ret;
    }
    // Add Environmental Sensor Service
    ret = Add_Environmental_Sensor_Service();
    if(ret == BLE_STATUS_SUCCESS) {
        PRINTF("Environmental service added successfully.\r\n");
    } else {
        PRINTF("Error while adding Environmental service: 0x%04x\r\n", ret);
        return ret;
    }
    #if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    ret = OTA_Add_Btl_Service();
    if(ret == BLE_STATUS_SUCCESS)
        PRINTF("OTA service added successfully.\r\n");
    else
    PRINTF("Error while adding OTA service.\r\n");
    #endif // ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    // Start the Sensor Timer
    ret = HAL_VTimerStart_ms(SENSOR_TIMER, acceleration_update_rate);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("HAL_VTimerStart_ms() failed; 0x%02x\r\n", ret);
        return ret;
    } else {
        sensorTimer_expired = FALSE;
    }

    */

    return BLE_STATUS_SUCCESS;
}


/******************************************************************************
 * Function Name  : setDeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *****************************************************************************/
void Blue1::setDeviceConnectable(void){
    uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','m','b','e','d'};

    #if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
        hci_le_set_scan_response_data(18,BTLServiceUUID4Scan);
    #else
        hci_le_set_scan_response_data(0,NULL);
    #endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

    PRINTF("Set General Discoverable Mode.\r\n");

    ret = aci_gap_set_discoverable(ADV_IND,
                                   (ADV_INTERVAL_MIN_MS*1000)/625,(ADV_INTERVAL_MAX_MS*1000)/625,
                                    STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                    sizeof(local_name), local_name, 0, NULL, 0, 0);
    if(ret != BLE_STATUS_SUCCESS){
        PRINTF("aci_gap_set_discoverable() failed: 0x%02x\r\n",ret);
        *led3 = 1;//SdkEvalLedOn(LED3);
    }
    else
        PRINTF("aci_gap_set_discoverable() --> SUCCESS\r\n");
}




/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/******************************************************************************/
extern "C" {
    void Blue_Handler(void){
        // Call RAL_Isr
        RAL_Isr();
    }

    /*void SysTick_Handler(void){
        SysCount_Handler();
    }*/
}