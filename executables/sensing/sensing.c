/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * 
 * This example uses ABP to join the LoRaWAN network and then sends a
 * "hello world" uplink message periodically and prints out the
 * contents of any downlink message.
 */

#ifdef DEBUG
    #include <stdio.h>
#endif
#include <string.h>


#include "pico/stdlib.h"
#include "../bme/bme68x/bme68x.h"
#include "../bme/bme_api/bme68x_API.h"
#include "../bme/bsec/bsec_datatypes.h"
#include "../bme/bsec/bsec_interface.h"

//fs
//#include "littlefs-lib/pico_hal.h"
#include "sd_card.h"
#include "ff.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "bme-config.h"

#define REQUESTED_OUTPUT        6
#define BME68X_VALID_DATA       UINT8_C(0xB0)
#define BSEC_CHECK_INPUT(x, shift)		(x & (1 << (shift-1)))
/*
    Variables handling the rtc sleep
    registers and clocks
*/
static uint scb_orig;
static uint clock0_orig;
static uint clock1_orig;
//measurements basically
bsec_sensor_configuration_t requested_virtual_sensors[REQUESTED_OUTPUT];
uint8_t n_requested_virtual_sensors = REQUESTED_OUTPUT;
// Allocate a struct for the returned physical sensor settings
bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR]; //should i put 1 since i have no shuttle?
uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
//configuration coming from bsec
bsec_bme_settings_t conf_bsec;

//getting state variables
uint8_t serialized_state_in[BSEC_MAX_PROPERTY_BLOB_SIZE];
uint32_t n_serialized_state_in = BSEC_MAX_PROPERTY_BLOB_SIZE;
uint8_t work_buffer_state_in[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer_size_in = BSEC_MAX_WORKBUFFER_SIZE;
//saving state variables
uint8_t serialized_state_out[BSEC_MAX_STATE_BLOB_SIZE];
uint32_t n_serialized_state_max_out = BSEC_MAX_STATE_BLOB_SIZE;
uint32_t n_serialized_state_out = BSEC_MAX_STATE_BLOB_SIZE;
uint8_t work_buffer_state_out[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer_size_out = BSEC_MAX_WORKBUFFER_SIZE;
//configuration on shut down
uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];
uint32_t n_serialized_settings_max = BSEC_MAX_PROPERTY_BLOB_SIZE;
uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer = BSEC_MAX_WORKBUFFER_SIZE;
/*
    File System variables
*/
FRESULT fr;
FATFS fs;
FIL fil;
UINT bread = 0;
UINT bwritten = 0;

/*
    BSEC VARIABLES
*/
bsec_library_return_t rslt_bsec;

/**
 * @brief utility to print out the results
 * 
 * @param id id of the data
 * @param signal value of the data
 * @param accuracy accuracy of the data
 */
void print_results(int id, float signal, int accuracy);

uint8_t processData(int64_t currTimeNs, const struct bme68x_data data, bsec_input_t* inputs){
    uint8_t n_input = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (conf_bsec.process_data & BSEC_PROCESS_TEMPERATURE)
    {
        inputs[n_input].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[n_input].signal = 0;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
        
        inputs[n_input].signal = data.temperature;
        inputs[n_input].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if (conf_bsec.process_data & BSEC_PROCESS_HUMIDITY)
    {
        inputs[n_input].signal = data.humidity;
        inputs[n_input].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if (conf_bsec.process_data & BSEC_PROCESS_PRESSURE)
    {
        inputs[n_input].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[n_input].signal = data.pressure;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if ((conf_bsec.process_data & BSEC_PROCESS_GAS) && (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[n_input].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[n_input].signal = data.gas_resistance;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if (BSEC_CHECK_INPUT(conf_bsec.process_data, BSEC_INPUT_PROFILE_PART) && (data.status & BME68X_GASM_VALID_MSK)){
        inputs[n_input].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[n_input].signal = (conf_bsec.op_mode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }

    return n_input;
}

int main( void )
{   
    
    //
    uint32_t time_us;
    /*
        BME API VARIABLES
    */
    struct bme68x_dev bme;
    struct bme68x_data data[3];
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    int8_t rslt_api;
    uint8_t n_fields;
    uint32_t del_period;
    uint16_t sample_count = 0;

    
    /*
        INITIALIZE GPIO PINS
    */

    // initialize stdio and wait for USB CDC connect
    stdio_uart_init();
    /*
        little FS set up, it mounts a file system on the flash memory to save the state file
        if set to true it formats everything
        if set to false it keeps the files as they were
    */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    gpio_init(PIN_FORMAT_OUTPUT);
    gpio_init(PIN_FORMAT_INPUT);
    gpio_set_dir(PIN_FORMAT_OUTPUT, GPIO_OUT);
    gpio_set_dir(PIN_FORMAT_INPUT, GPIO_IN);
    gpio_put(PIN_FORMAT_OUTPUT, 1);
    
    sleep_ms(50);
    bool format = gpio_get(PIN_FORMAT_INPUT);
#ifdef DEBUG
    printf("...mounting FS");
    format ? printf(" and formatting\n") : printf("\n");
#endif
    if (!sd_init_driver()) {
    #ifdef DEBUG
        printf("ERROR: Could not initialize SD card\r\n");
    #endif
        blink();
    }

    
    gpio_put(PIN_FORMAT_OUTPUT, 0);
#ifdef DEBUG
    printf("...initialization BSEC\n");
#endif
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_1;
    requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_2;
    requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_SCAN;
   
    /*
        INITIALIZATION BME CONFIGURATION
    */
#ifdef DEBUG
    printf("...initialization BME688\n");
#endif
    bme_interface_init(&bme, BME68X_I2C_INTF);
    uint8_t data_id;

    //read device id after init to see if everything works for now and to check that the device communicates with I2C
    bme.read(BME68X_REG_CHIP_ID, &data_id, 1, bme.intf_ptr);
    if(data_id != BME68X_CHIP_ID){
    #ifdef DEBUG
        printf("Cannot communicate with BME688\n");
        printf("CHIP_ID: %x \t ID_READ: %x\n", BME68X_CHIP_ID, data_id);
    #endif
        blink();
    }
    else{
        bme.chip_id = data_id;
    #ifdef DEBUG
        printf("Connection valid, DEVICE_ID: %x\n", bme.chip_id);
    #endif
    }


    /*
        initialize the structure with all the parameters by reading from the registers
    */
    rslt_api = bme68x_init(&bme);
    check_rslt_api( rslt_api, "INIT");

    //read state to get the previous state and avoid restarting everything
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
    #ifdef DEBUG
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
    #endif
        blink();
    }

    //open settings file and set
    fr = f_open(&fil, "prova.txt", FA_READ);
    if(fr != FR_OK && fr != FR_NO_FILE){
    #ifdef DEBUG
        printf("Error opening the file\n");
    #endif
        blink();    
    }
    char prova[2048];
    if(fr == FR_OK){
        fr = f_read(&fil, prova, 2048*sizeof(char), &bread);
        if(fr != FR_OK){
        #ifdef DEBUG
            printf("Error reading the file\n");
        #endif
            blink();
        }
    }
    if(fr == FR_NO_FILE){
        #ifdef DEBUG
            printf("There is no settings file saved\n");
        #endif
    }
    f_close(&fil);
    if(bread > 0){
        prova[bread] = '\0';
        printf("%s\n", prova);
    }

    fr = f_open(&fil, config_file_name, FA_READ);
    if(fr != FR_OK && fr != FR_NO_FILE){
    #ifdef DEBUG
        printf("Error opening the file\n");
    #endif
        blink();    
    }
    uint8_t ss[2048];

    if(fr == FR_OK){
        fr = f_read(&fil, serialized_settings, BSEC_MAX_PROPERTY_BLOB_SIZE*sizeof(uint8_t), &bread);
        if(fr != FR_OK){
        #ifdef DEBUG
            printf("Error reading the file\n");
        #endif
            blink();
        }
    }
    if(fr == FR_NO_FILE){
        #ifdef DEBUG
            printf("There is no settings file saved\n");
        #endif
    }
    f_close(&fil);
    rslt_bsec = bsec_init();
    check_rslt_bsec( rslt_bsec, "BSEC_INIT");
    /*
        INITIALIZATION BSEC LIBRARY
    */
#ifdef DEBUG
    bsec_version_t v;
    bsec_get_version(&v);
    printf("Version: %d.%d.%d.%d\n", v.major, v.minor, v.major_bugfix, v.minor_bugfix);
#endif
    if(bread > 0){
    #ifdef DEBUG
        printf("...loading configuration, read %d bytes\n", bread);
        //const uint8_t bsec_config_selectivity[2285] = {0,0,2,2,189,1,0,0,0,0,0,0,213,8,0,0,52,0,1,0,0,192,168,71,64,49,119,76,0,0,97,69,0,0,97,69,137,65,0,63,0,0,0,63,0,0,64,63,205,204,204,62,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,35,41,29,86,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,173,6,11,0,1,2,2,207,61,208,65,149,110,24,66,180,108,177,65,219,148,13,192,70,132,58,66,163,58,140,192,12,99,178,192,185,59,255,193,178,213,175,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,110,211,55,195,237,6,56,67,0,0,0,0,0,0,0,0,26,3,255,63,61,246,7,192,171,201,228,192,249,144,215,63,237,199,104,193,128,158,216,64,117,140,155,63,137,120,129,67,108,109,253,191,0,0,0,0,81,64,57,68,103,241,231,64,109,29,152,192,209,43,193,63,93,54,30,65,197,46,92,64,128,27,224,192,6,20,144,191,56,179,130,64,0,0,0,0,43,156,59,196,33,217,100,194,104,77,72,65,15,103,175,191,249,252,12,193,63,117,253,192,233,5,141,65,155,42,25,64,13,88,249,191,0,0,0,0,48,141,122,190,204,150,44,192,36,162,29,193,96,59,39,189,54,202,48,65,151,205,68,64,79,105,55,193,53,120,53,192,77,211,32,192,0,0,0,0,193,207,92,65,239,201,76,65,208,70,82,66,81,63,96,65,48,179,0,194,251,96,242,193,176,51,96,194,153,114,98,66,144,247,64,65,0,0,0,0,219,179,180,63,175,218,119,191,51,71,207,191,245,145,129,63,53,16,244,65,138,208,117,65,138,97,36,66,228,15,32,195,126,91,103,191,0,0,0,0,26,151,170,193,64,105,49,193,46,223,189,193,129,203,168,193,40,91,49,66,4,87,107,65,205,202,37,65,244,36,154,66,240,85,39,193,0,0,0,0,166,96,87,192,114,7,68,191,233,32,214,63,84,249,40,192,45,78,132,64,145,33,253,61,49,43,187,192,244,32,77,67,224,250,71,191,0,0,0,0,103,75,214,190,206,141,252,63,99,15,178,65,80,79,166,190,214,25,146,192,165,29,24,194,18,228,219,193,113,246,235,194,49,115,232,63,0,0,0,0,17,211,124,64,56,252,251,62,25,118,148,193,168,234,94,64,131,157,82,64,217,119,236,65,120,245,240,65,17,69,168,195,49,51,8,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,131,217,97,66,182,104,101,194,0,0,0,0,0,0,0,0,6,142,142,195,229,54,143,67,0,0,0,0,0,0,0,0,25,224,153,66,217,51,154,194,0,0,0,0,0,0,0,0,142,36,105,194,199,63,110,66,0,0,0,0,0,0,0,0,206,73,250,193,138,69,249,65,0,0,0,0,0,0,0,0,123,173,127,66,20,116,128,194,0,0,0,0,0,0,0,0,49,65,49,64,205,213,107,192,0,0,0,0,0,0,0,0,189,250,179,194,164,98,180,66,0,0,0,0,0,0,0,0,96,182,197,67,155,71,197,195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,185,28,179,73,64,25,141,76,223,26,138,76,222,207,133,76,87,134,164,75,23,127,159,75,166,9,155,75,94,120,170,73,95,221,177,73,93,44,182,73,0,0,0,0,0,0,0,0,0,0,0,0,30,55,120,73,215,98,32,76,7,79,34,76,161,238,36,76,119,151,160,75,119,96,157,75,202,75,154,75,118,89,111,73,133,239,116,73,219,140,120,73,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,87,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,0,0,0,0,145,33,0,0};
        //const uint8_t bsec_config_selectivity[1974] = {0,0,4,2,189,1,0,0,0,0,0,0,158,7,0,0,176,0,1,0,0,192,168,71,64,49,119,76,0,0,97,69,0,0,97,69,137,65,0,63,0,0,0,63,0,0,64,63,205,204,204,62,10,0,3,0,0,0,96,64,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,205,204,204,189,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,128,63,82,73,157,188,95,41,203,61,118,224,108,63,155,230,125,63,191,14,124,63,0,0,160,65,0,0,32,66,0,0,160,65,0,0,32,66,0,0,32,66,0,0,160,65,0,0,32,66,0,0,160,65,8,0,2,0,236,81,133,66,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,35,41,29,86,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,13,5,11,0,1,2,2,207,61,208,65,149,110,24,66,180,108,177,65,219,148,13,192,70,132,58,66,163,58,140,192,12,99,178,192,185,59,255,193,178,213,175,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,110,211,55,195,237,6,56,67,0,0,0,0,0,0,0,0,26,3,255,63,61,246,7,192,171,201,228,192,249,144,215,63,237,199,104,193,128,158,216,64,117,140,155,63,137,120,129,67,108,109,253,191,0,0,0,0,81,64,57,68,103,241,231,64,109,29,152,192,209,43,193,63,93,54,30,65,197,46,92,64,128,27,224,192,6,20,144,191,56,179,130,64,0,0,0,0,43,156,59,196,33,217,100,194,104,77,72,65,15,103,175,191,249,252,12,193,63,117,253,192,233,5,141,65,155,42,25,64,13,88,249,191,0,0,0,0,48,141,122,190,204,150,44,192,36,162,29,193,96,59,39,189,54,202,48,65,151,205,68,64,79,105,55,193,53,120,53,192,77,211,32,192,0,0,0,0,193,207,92,65,239,201,76,65,208,70,82,66,81,63,96,65,48,179,0,194,251,96,242,193,176,51,96,194,153,114,98,66,144,247,64,65,0,0,0,0,219,179,180,63,175,218,119,191,51,71,207,191,245,145,129,63,53,16,244,65,138,208,117,65,138,97,36,66,228,15,32,195,126,91,103,191,0,0,0,0,26,151,170,193,64,105,49,193,46,223,189,193,129,203,168,193,40,91,49,66,4,87,107,65,205,202,37,65,244,36,154,66,240,85,39,193,0,0,0,0,166,96,87,192,114,7,68,191,233,32,214,63,84,249,40,192,45,78,132,64,145,33,253,61,49,43,187,192,244,32,77,67,224,250,71,191,0,0,0,0,103,75,214,190,206,141,252,63,99,15,178,65,80,79,166,190,214,25,146,192,165,29,24,194,18,228,219,193,113,246,235,194,49,115,232,63,0,0,0,0,17,211,124,64,56,252,251,62,25,118,148,193,168,234,94,64,131,157,82,64,217,119,236,65,120,245,240,65,17,69,168,195,49,51,8,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,131,217,97,66,182,104,101,194,0,0,0,0,0,0,0,0,6,142,142,195,229,54,143,67,0,0,0,0,0,0,0,0,25,224,153,66,217,51,154,194,0,0,0,0,0,0,0,0,142,36,105,194,199,63,110,66,0,0,0,0,0,0,0,0,206,73,250,193,138,69,249,65,0,0,0,0,0,0,0,0,123,173,127,66,20,116,128,194,0,0,0,0,0,0,0,0,49,65,49,64,205,213,107,192,0,0,0,0,0,0,0,0,189,250,179,194,164,98,180,66,0,0,0,0,0,0,0,0,96,182,197,67,155,71,197,195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,185,28,179,73,64,25,141,76,223,26,138,76,222,207,133,76,87,134,164,75,23,127,159,75,166,9,155,75,94,120,170,73,95,221,177,73,93,44,182,73,0,0,0,0,0,0,0,0,0,0,0,0,30,55,120,73,215,98,32,76,7,79,34,76,161,238,36,76,119,151,160,75,119,96,157,75,202,75,154,75,118,89,111,73,133,239,116,73,219,140,120,73,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,88,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,1,0,0,0,0,233,74,0,0};
        const uint8_t bsec_config_selectivity[2285] = {0,0,2,2,189,1,0,0,0,0,0,0,213,8,0,0,52,0,1,0,0,192,168,71,64,49,119,76,0,0,97,69,0,0,97,69,137,65,0,63,0,0,0,63,0,0,64,63,205,204,204,62,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,35,41,29,86,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,173,6,11,0,1,2,2,207,61,208,65,149,110,24,66,180,108,177,65,219,148,13,192,70,132,58,66,163,58,140,192,12,99,178,192,185,59,255,193,178,213,175,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,110,211,55,195,237,6,56,67,0,0,0,0,0,0,0,0,26,3,255,63,61,246,7,192,171,201,228,192,249,144,215,63,237,199,104,193,128,158,216,64,117,140,155,63,137,120,129,67,108,109,253,191,0,0,0,0,81,64,57,68,103,241,231,64,109,29,152,192,209,43,193,63,93,54,30,65,197,46,92,64,128,27,224,192,6,20,144,191,56,179,130,64,0,0,0,0,43,156,59,196,33,217,100,194,104,77,72,65,15,103,175,191,249,252,12,193,63,117,253,192,233,5,141,65,155,42,25,64,13,88,249,191,0,0,0,0,48,141,122,190,204,150,44,192,36,162,29,193,96,59,39,189,54,202,48,65,151,205,68,64,79,105,55,193,53,120,53,192,77,211,32,192,0,0,0,0,193,207,92,65,239,201,76,65,208,70,82,66,81,63,96,65,48,179,0,194,251,96,242,193,176,51,96,194,153,114,98,66,144,247,64,65,0,0,0,0,219,179,180,63,175,218,119,191,51,71,207,191,245,145,129,63,53,16,244,65,138,208,117,65,138,97,36,66,228,15,32,195,126,91,103,191,0,0,0,0,26,151,170,193,64,105,49,193,46,223,189,193,129,203,168,193,40,91,49,66,4,87,107,65,205,202,37,65,244,36,154,66,240,85,39,193,0,0,0,0,166,96,87,192,114,7,68,191,233,32,214,63,84,249,40,192,45,78,132,64,145,33,253,61,49,43,187,192,244,32,77,67,224,250,71,191,0,0,0,0,103,75,214,190,206,141,252,63,99,15,178,65,80,79,166,190,214,25,146,192,165,29,24,194,18,228,219,193,113,246,235,194,49,115,232,63,0,0,0,0,17,211,124,64,56,252,251,62,25,118,148,193,168,234,94,64,131,157,82,64,217,119,236,65,120,245,240,65,17,69,168,195,49,51,8,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,131,217,97,66,182,104,101,194,0,0,0,0,0,0,0,0,6,142,142,195,229,54,143,67,0,0,0,0,0,0,0,0,25,224,153,66,217,51,154,194,0,0,0,0,0,0,0,0,142,36,105,194,199,63,110,66,0,0,0,0,0,0,0,0,206,73,250,193,138,69,249,65,0,0,0,0,0,0,0,0,123,173,127,66,20,116,128,194,0,0,0,0,0,0,0,0,49,65,49,64,205,213,107,192,0,0,0,0,0,0,0,0,189,250,179,194,164,98,180,66,0,0,0,0,0,0,0,0,96,182,197,67,155,71,197,195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,185,28,179,73,64,25,141,76,223,26,138,76,222,207,133,76,87,134,164,75,23,127,159,75,166,9,155,75,94,120,170,73,95,221,177,73,93,44,182,73,0,0,0,0,0,0,0,0,0,0,0,0,30,55,120,73,215,98,32,76,7,79,34,76,161,238,36,76,119,151,160,75,119,96,157,75,202,75,154,75,118,89,111,73,133,239,116,73,219,140,120,73,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,87,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,0,0,0,0,145,33,0,0};

    #endif
        rslt_bsec = bsec_set_configuration(bsec_config_selectivity, n_serialized_settings_max, work_buffer, n_work_buffer);
        check_rslt_bsec( rslt_bsec, "BSEC_SET_CONFIGURATION");
    }
    bread = 0;
    //state file operations
    if(!format){//if not format mount and read, otherwise avoid
        printf("Loading state\n");
        bread = 0;
        fr = f_open(&fil, state_file_name, FA_READ);
        if(fr != FR_OK && fr != FR_NO_FILE){
        #ifdef DEBUG
            printf("Error opening the file\n");
        #endif
            blink();    
        }
        printf("Opened state\n");

        if(fr == FR_OK){
            fr = f_read(&fil, serialized_state_in, BSEC_MAX_WORKBUFFER_SIZE*sizeof(uint8_t), &bread);
            if(fr != FR_OK){
            #ifdef DEBUG
                printf("Error reading the file\n");
            #endif
                blink();
            }
        }
        printf("Ok state\n");

        if(fr == FR_NO_FILE){
            #ifdef DEBUG
                printf("There is no state file saved\n");
            #endif
        }
        f_close(&fil);
        printf("closed\n");

    }

    f_unmount("0:");
    
    //if there is a state file saved somewhere it loads the variable back
    if(bread > 0){
    #ifdef DEBUG
        printf("...resuming the state, read %d bytes\n", bread);
    #endif
        bsec_set_state(serialized_state_in, n_serialized_state_in, work_buffer_state_in, n_work_buffer_size_in);
        check_rslt_bsec( rslt_bsec, "BSEC_SET_STATE");
    }


    sleep_ms(1000);
    gpio_deinit(PIN_FORMAT_INPUT);
    gpio_deinit(PIN_FORMAT_OUTPUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // Call bsec_update_subscription() to enable/disable the requested virtual sensors
    rslt_bsec = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);
    check_rslt_bsec( rslt_bsec, "BSEC_UPDATE_SUBSCRIPTION");
    uint8_t current_op_mode = 0;
    // loop forever
    conf_bsec.next_call = 0;
    while (1) {
        sample_count = 0;
        //set to forced mode
        if(time_us_64()*1000 >= conf_bsec.next_call){
            rslt_bsec = bsec_sensor_control(time_us_64()*1000, &conf_bsec);
            check_rslt_bsec(rslt_bsec, "BSEC_SENSOR_CONTROL");
        }
        
        if(conf_bsec.op_mode != current_op_mode){
            /*
                Set oversampling for measurements
            */
            conf.os_hum = conf_bsec.humidity_oversampling;
            conf.os_pres = conf_bsec.pressure_oversampling;
            conf.os_temp = conf_bsec.temperature_oversampling;
            conf.filter = BME68X_FILTER_OFF;
            conf.odr = BME68X_ODR_NONE;

            /*  
                Set the remaining gas sensor settings and link the heating profile 
                enable the heater plate
            */
            heatr_conf.enable = conf_bsec.run_gas;
            rslt_api = bme68x_set_conf(&conf, &bme);
            check_rslt_api(rslt_api, "bme68x_set_conf");

            if(conf_bsec.op_mode == BME68X_FORCED_MODE){
                printf("--------------Forced Mode--------------\n");
                heatr_conf.heatr_temp = conf_bsec.heater_temperature;
                heatr_conf.heatr_dur = conf_bsec.heater_duration;
                current_op_mode = BME68X_FORCED_MODE;

                

            }

            if(conf_bsec.op_mode == BME68X_PARALLEL_MODE){
                printf("--------------Parallel Mode--------------\n");
                heatr_conf.heatr_dur_prof = conf_bsec.heater_duration_profile;
                heatr_conf.heatr_temp_prof = conf_bsec.heater_temperature_profile;
                heatr_conf.profile_len = 10;
                heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000);
                current_op_mode = BME68X_PARALLEL_MODE;

            }
            
            rslt_api = bme68x_set_heatr_conf(conf_bsec.op_mode, &heatr_conf, &bme);
            check_rslt_api(rslt_api, "bme68x_set_heatr_conf");

            rslt_api = bme68x_set_op_mode(conf_bsec.op_mode, &bme); 
            check_rslt_api(rslt_api, "bme68x_set_op_mode");
        }
        if(conf_bsec.trigger_measurement == 1){
            if(conf_bsec.op_mode == BME68X_FORCED_MODE)
                del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
                
            
            if(conf_bsec.op_mode == BME68X_PARALLEL_MODE) 
                del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);

            if(conf_bsec.trigger_measurement == 1){
                //compute delay period for heated plate
                //delay
                bme.delay_us(del_period, bme.intf_ptr);
                rslt_api = bme68x_get_op_mode(&current_op_mode, &bme);
                check_rslt_api(rslt_api, "bme68x_get_op_mode");
                //get data
                rslt_api = bme68x_get_data(conf_bsec.op_mode, data, &n_fields, &bme);
                check_rslt_api(rslt_api, "bme68x_get_data");
            }
            
            if(conf_bsec.process_data != 0){
                uint8_t n_input = 0;
                bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
                printf("Process data\n");
                for(int i = 0; i<n_fields; i++){
                    if(data[i].status == BME68X_VALID_DATA){
                        printf("Sample, TimeStamp(ms), Bits, Status, Gas index, Meas index\n");
                        printf("%u, %lu, %lu, 0x%x, %d, %d\n",
                        sample_count,
                        (long unsigned int)time_us,
                        conf_bsec.process_data,
                        data[i].status,
                        data[i].gas_index,
                        data[i].meas_index);
                        time_us = time_us_32()*1000;
                        n_input = processData(time_us, data[i], inputs);
                        if(n_input > 0){
                            uint8_t n_output = REQUESTED_OUTPUT;
                            bsec_output_t output[BSEC_NUMBER_OUTPUTS];

                            rslt_bsec = bsec_do_steps(inputs, n_input, output, &n_output);
                            if(rslt_bsec == BSEC_OK){
                                for(uint8_t  i = 0; i < n_output; i++){
                                #ifdef DEBUG
                                    print_results(output[i].sensor_id, output[i].signal, output[i].accuracy);
                                #endif
                                }
                            }
                        }
                    }
                }
            } 
            sample_count++;    
        }
        if(current_op_mode != conf_bsec.op_mode){
            printf("Resetting op mode to sleep\n");
            rslt_api = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme); 
            check_rslt_api(rslt_api, "bme68x_set_op_mode");
            current_op_mode = 0;
        } 
    }
    save_state_file();
    return 0;
}

//utility to print results
void print_results(int id, float signal, int accuracy){
    switch(id){
        case BSEC_OUTPUT_IAQ:
            printf("IAQ\n");
            printf("%.2f  %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_STATIC_IAQ:
            printf("STATIC IAQ\n");
            printf("%.2f   %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            printf("CO2[ppm]\n");
            printf("%.2f   %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            printf("VOC\n");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RAW_TEMPERATURE:
                printf("Temperature[°C]\n");
                printf("%.2f \n", signal);
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
                printf("Pressure[Pa]\n");
                printf("%.2f \n", signal);
            break;
        case BSEC_OUTPUT_RAW_HUMIDITY:
                printf("Humidity[%%rH]\n");
                printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RAW_GAS:
            printf("[Ohm]\n");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            printf("Stabilization\n");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            printf("Run in\n");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            printf("Compensated T\n");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            printf("Compensated H\n");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_GAS_PERCENTAGE:
            printf("Gas[%%]\n");
            printf("%.2f %d\n", signal, accuracy);
            break;
    }
}

void save_state_file(){
    //SAVING THE FILE ON THE FILESYSTEM
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    printf("..Saving the file and shutting down\n");
    fr = f_mount(&fs, "0:", 1);
        if (fr != FR_OK) {
            printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
            blink();
        }
    fr = f_open(&fil, state_file_name, FA_WRITE | FA_CREATE_ALWAYS);
    if(fr != FR_OK){
            printf("ERROR: Could not create file (%d)\r\n", fr);
            blink();
    }
    rslt_bsec = bsec_get_state(0, serialized_state_out, n_serialized_state_max_out, work_buffer_state_out, n_work_buffer_size_out, &n_serialized_state_out);
    check_rslt_bsec(rslt_bsec, "BSEC_GET_STATE");
    fr = f_write(&fil, serialized_state_out, BSEC_MAX_PROPERTY_BLOB_SIZE*sizeof(uint8_t), &bwritten);
    if(fr != FR_OK){
            printf("ERROR: Could not write file (%d)\r\n", fr);
            blink();
    }
    f_close(&fil);
    if(fr != FR_OK){
            printf("ERROR: Could not close file (%d)\r\n", fr);
            blink();
    }

    f_unmount("0:");
    //turn off the led, system can be shut down 
    printf("Written %d byte for file %s\n", bwritten, state_file_name);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}
