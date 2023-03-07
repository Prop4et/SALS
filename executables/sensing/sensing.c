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
#include "../lib/bme/bme68x/bme68x.h"
#include "../lib/bme/bme_api/bme68x_API.h"
#include "../lib/bme/bsec/bsec_datatypes.h"
#include "../lib/bme/bsec/bsec_interface.h"

//fs
//#include "littlefs-lib/pico_hal.h"
#include "sd_card.h"
#include "ff.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "bme-config.h"
#include "hardware/watchdog.h"

#define REQUESTED_OUTPUT        2
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
 * @brief watchdog that goes on a loop to force a reset of the pico
 * 
 */
void software_reset()
{
    watchdog_enable(1, 1);
    while(1);
}


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
    
    uint8_t not_sent_loops = 0;
    uint32_t last_saved = 0;
    /*
        PKT AND CONSTANT VALUES
    */
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
    uint8_t n_fields = 0;

    uint32_t del_period;
    
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
    
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_1;
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_2;
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    
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
        //const uint8_t bsec_config_selectivity[2285] = {0,0,2,2,189,1,0,0,0,0,0,0,213,8,0,0,52,0,1,0,0,168,19,73,64,49,119,76,0,192,40,72,0,192,40,72,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,254,131,137,87,88,0,9,0,7,240,150,61,0,0,0,0,0,0,0,0,28,124,225,61,52,128,215,63,0,0,160,64,0,0,0,0,0,0,0,0,205,204,12,62,103,213,39,62,230,63,76,192,0,0,0,0,0,0,0,0,145,237,60,191,251,58,64,63,177,80,131,64,0,0,0,0,0,0,0,0,93,254,227,62,54,60,133,191,0,0,64,64,12,0,10,0,0,0,0,0,0,0,0,0,173,6,11,0,0,0,2,238,156,35,63,228,238,20,63,36,25,29,190,92,118,35,62,242,141,110,62,205,136,161,62,183,129,44,61,122,186,32,190,192,42,23,190,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,151,170,146,62,151,170,146,190,0,0,0,0,0,0,0,0,207,114,149,190,200,227,27,191,63,137,177,190,203,149,145,189,92,177,185,61,160,248,151,190,131,31,177,190,118,119,214,62,152,45,158,190,0,0,0,0,57,107,238,190,121,63,79,61,168,134,124,190,139,192,132,61,96,180,211,62,231,73,213,190,161,21,97,62,92,155,65,190,193,171,8,191,0,0,0,0,110,26,48,191,139,194,42,191,12,161,184,62,230,198,174,62,196,167,110,62,1,228,231,61,190,136,82,190,221,69,168,190,78,198,206,61,0,0,0,0,224,167,122,190,229,172,176,190,199,3,7,191,67,153,63,62,158,254,8,63,26,43,32,187,121,233,142,189,153,76,43,62,187,93,105,62,0,0,0,0,72,121,120,190,158,198,136,189,93,147,143,190,39,131,94,62,188,15,76,63,18,136,133,190,53,62,40,190,71,193,41,189,213,202,143,189,0,0,0,0,63,245,93,62,38,73,197,62,227,159,51,190,129,0,25,62,183,119,25,190,37,66,35,189,132,83,130,190,27,34,77,62,233,174,66,191,0,0,0,0,22,76,146,187,2,47,134,190,41,60,78,62,113,20,247,62,195,126,254,62,133,139,38,190,227,159,203,189,187,95,229,190,62,97,223,189,0,0,0,0,191,222,104,63,141,65,227,62,185,5,7,191,134,18,250,62,205,248,45,190,25,119,61,63,29,78,189,61,196,50,165,62,187,84,224,190,0,0,0,0,211,168,215,62,52,174,57,63,89,224,125,190,115,148,75,189,170,38,201,62,120,54,208,61,77,75,25,191,212,203,210,62,222,90,81,191,0,0,0,0,36,15,120,63,213,52,78,63,133,182,46,190,184,157,138,61,204,128,30,63,196,139,236,62,200,29,46,191,235,48,173,190,246,79,192,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,130,32,184,62,171,0,113,190,0,0,0,0,0,0,0,0,180,133,213,62,14,227,129,190,0,0,0,0,0,0,0,0,74,98,216,189,175,40,67,63,0,0,0,0,0,0,0,0,113,31,132,63,111,53,38,190,0,0,0,0,0,0,0,0,163,130,178,60,199,146,216,190,0,0,0,0,0,0,0,0,27,127,4,63,243,150,45,191,0,0,0,0,0,0,0,0,57,165,5,191,222,5,59,63,0,0,0,0,0,0,0,0,112,75,126,190,75,38,169,189,0,0,0,0,0,0,0,0,165,170,132,190,106,233,29,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,200,143,253,71,203,229,146,75,172,226,118,75,5,216,68,75,131,159,38,73,45,226,23,73,0,152,12,73,245,78,210,71,66,227,255,71,27,80,13,72,0,0,0,0,0,0,0,0,0,0,0,0,7,196,242,71,158,185,172,75,215,247,144,75,19,168,100,75,178,182,50,73,72,145,33,73,225,122,20,73,180,36,187,71,225,235,223,71,60,84,245,71,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,87,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,0,0,0,0,137,26,0,0};
    #endif
        //rslt_bsec = bsec_set_configuration(bsec_config_selectivity, n_serialized_settings_max, work_buffer, n_work_buffer);
        //check_rslt_bsec( rslt_bsec, "BSEC_SET_CONFIGURATION");
    }
    bread = 0;
    const uint8_t bsec_config_selectivity[2285] = {0,0,2,2,189,1,0,0,0,0,0,0,213,8,0,0,52,0,1,0,0,168,19,73,64,49,119,76,0,192,40,72,0,192,40,72,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,254,131,137,87,88,0,9,0,7,240,150,61,0,0,0,0,0,0,0,0,28,124,225,61,52,128,215,63,0,0,160,64,0,0,0,0,0,0,0,0,205,204,12,62,103,213,39,62,230,63,76,192,0,0,0,0,0,0,0,0,145,237,60,191,251,58,64,63,177,80,131,64,0,0,0,0,0,0,0,0,93,254,227,62,54,60,133,191,0,0,64,64,12,0,10,0,0,0,0,0,0,0,0,0,173,6,11,0,0,0,2,238,156,35,63,228,238,20,63,36,25,29,190,92,118,35,62,242,141,110,62,205,136,161,62,183,129,44,61,122,186,32,190,192,42,23,190,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,151,170,146,62,151,170,146,190,0,0,0,0,0,0,0,0,207,114,149,190,200,227,27,191,63,137,177,190,203,149,145,189,92,177,185,61,160,248,151,190,131,31,177,190,118,119,214,62,152,45,158,190,0,0,0,0,57,107,238,190,121,63,79,61,168,134,124,190,139,192,132,61,96,180,211,62,231,73,213,190,161,21,97,62,92,155,65,190,193,171,8,191,0,0,0,0,110,26,48,191,139,194,42,191,12,161,184,62,230,198,174,62,196,167,110,62,1,228,231,61,190,136,82,190,221,69,168,190,78,198,206,61,0,0,0,0,224,167,122,190,229,172,176,190,199,3,7,191,67,153,63,62,158,254,8,63,26,43,32,187,121,233,142,189,153,76,43,62,187,93,105,62,0,0,0,0,72,121,120,190,158,198,136,189,93,147,143,190,39,131,94,62,188,15,76,63,18,136,133,190,53,62,40,190,71,193,41,189,213,202,143,189,0,0,0,0,63,245,93,62,38,73,197,62,227,159,51,190,129,0,25,62,183,119,25,190,37,66,35,189,132,83,130,190,27,34,77,62,233,174,66,191,0,0,0,0,22,76,146,187,2,47,134,190,41,60,78,62,113,20,247,62,195,126,254,62,133,139,38,190,227,159,203,189,187,95,229,190,62,97,223,189,0,0,0,0,191,222,104,63,141,65,227,62,185,5,7,191,134,18,250,62,205,248,45,190,25,119,61,63,29,78,189,61,196,50,165,62,187,84,224,190,0,0,0,0,211,168,215,62,52,174,57,63,89,224,125,190,115,148,75,189,170,38,201,62,120,54,208,61,77,75,25,191,212,203,210,62,222,90,81,191,0,0,0,0,36,15,120,63,213,52,78,63,133,182,46,190,184,157,138,61,204,128,30,63,196,139,236,62,200,29,46,191,235,48,173,190,246,79,192,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,130,32,184,62,171,0,113,190,0,0,0,0,0,0,0,0,180,133,213,62,14,227,129,190,0,0,0,0,0,0,0,0,74,98,216,189,175,40,67,63,0,0,0,0,0,0,0,0,113,31,132,63,111,53,38,190,0,0,0,0,0,0,0,0,163,130,178,60,199,146,216,190,0,0,0,0,0,0,0,0,27,127,4,63,243,150,45,191,0,0,0,0,0,0,0,0,57,165,5,191,222,5,59,63,0,0,0,0,0,0,0,0,112,75,126,190,75,38,169,189,0,0,0,0,0,0,0,0,165,170,132,190,106,233,29,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,200,143,253,71,203,229,146,75,172,226,118,75,5,216,68,75,131,159,38,73,45,226,23,73,0,152,12,73,245,78,210,71,66,227,255,71,27,80,13,72,0,0,0,0,0,0,0,0,0,0,0,0,7,196,242,71,158,185,172,75,215,247,144,75,19,168,100,75,178,182,50,73,72,145,33,73,225,122,20,73,180,36,187,71,225,235,223,71,60,84,245,71,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,87,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,0,0,0,0,137,26,0,0};

    rslt_bsec = bsec_set_configuration(bsec_config_selectivity, n_serialized_settings_max, work_buffer, n_work_buffer);
    check_rslt_bsec( rslt_bsec, "BSEC_SET_CONFIGURATION");

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
    uint8_t current_op_mode = BME68X_SLEEP_MODE;

    // loop forever
    uint32_t sleep_time = 0;
    uint64_t start_time = time_us_64();
    while (1) {
        uint8_t nFieldsLeft = 0;
        uint64_t currTimeNs = time_us_64()*1000;
        current_op_mode = conf_bsec.op_mode;
        //set to forced mode
        if(currTimeNs >= conf_bsec.next_call){
            rslt_bsec = bsec_sensor_control(currTimeNs, &conf_bsec);
            check_rslt_bsec(rslt_bsec, "BSEC_SENSOR_CONTROL");
            if(rslt_bsec != BSEC_OK)
                continue;
            switch(conf_bsec.op_mode){
                case BME68X_FORCED_MODE:
                    printf("--------------Forced Mode--------------\n");
                    rslt_api = bme68x_get_conf(&conf, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_conf");
                    conf.os_hum = conf_bsec.humidity_oversampling;
                    conf.os_pres = conf_bsec.pressure_oversampling;
                    conf.os_temp = conf_bsec.temperature_oversampling;
                    conf.filter = BME68X_FILTER_OFF;
                    conf.odr = BME68X_ODR_NONE;
                    rslt_api = bme68x_set_conf(&conf, &bme);
                    check_rslt_api(rslt_api, "bme68x_set_conf");

                    heatr_conf.enable = BME68X_ENABLE;
                    heatr_conf.heatr_temp = conf_bsec.heater_temperature;
                    heatr_conf.heatr_dur = conf_bsec.heater_duration;
                    rslt_api = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
                    check_rslt_api(rslt_api, "bme68x_set_heatr_conf");

                    rslt_api = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme); 
                    check_rslt_api(rslt_api, "bme68x_set_op_mode");
                    current_op_mode = BME68X_FORCED_MODE;
                    break;
                case BME68X_PARALLEL_MODE:
                    if (current_op_mode != conf_bsec.op_mode){
                        printf("--------------Parallel Mode--------------\n");
                        rslt_api = bme68x_get_conf(&conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_get_conf");
                        conf.os_hum = conf_bsec.humidity_oversampling;
                        conf.os_pres = conf_bsec.pressure_oversampling;
                        conf.os_temp = conf_bsec.temperature_oversampling;
                        conf.filter = BME68X_FILTER_OFF;
                        conf.odr = BME68X_ODR_NONE;
                        rslt_api = bme68x_set_conf(&conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_conf");

                        heatr_conf.enable = BME68X_ENABLE;
                        heatr_conf.heatr_temp_prof = conf_bsec.heater_temperature_profile;
                        heatr_conf.heatr_dur_prof = conf_bsec.heater_duration_profile;
                        heatr_conf.profile_len = 10;
                        heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000);
                        current_op_mode = BME68X_PARALLEL_MODE;
                        rslt_api = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_heatr_conf");

                        rslt_api = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme); 
                        check_rslt_api(rslt_api, "bme68x_set_op_mode");
                        current_op_mode = BME68X_PARALLEL_MODE;
                    }
                    break;
                case BME68X_SLEEP_MODE:
                    if (current_op_mode != conf_bsec.op_mode){
                        printf("--------------Sleep Mode--------------\n");
                        rslt_api = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme); 
                        current_op_mode = BME68X_SLEEP_MODE;
                    }
                    break;
            }

            if(conf_bsec.trigger_measurement && conf_bsec.op_mode != BME68X_SLEEP_MODE){
                if(conf_bsec.op_mode == BME68X_FORCED_MODE){
                    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
                    bme.delay_us(del_period, bme.intf_ptr);
                    rslt_api = bme68x_get_op_mode(&current_op_mode, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_op_mode");
                    while (current_op_mode == BME68X_FORCED_MODE){
                        delay_us(5 * 1000, bme.intf_ptr);
                        rslt_api = bme68x_get_op_mode(&current_op_mode, &bme);
                    }
                    rslt_api = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_data");
                    if(data[0].status & BME68X_GASM_VALID_MSK){
                        uint8_t n_input = 0;
                        bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
                        n_input = processData(currTimeNs, data[0], inputs);
                        if(n_input > 0){
                            uint8_t n_output = REQUESTED_OUTPUT;
                            bsec_output_t output[BSEC_NUMBER_OUTPUTS];
                            memset(output, 0, sizeof(output));
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

                if(conf_bsec.op_mode == BME68X_PARALLEL_MODE){
                    del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);
                    bme.delay_us(del_period, bme.intf_ptr);
                    
                    rslt_api = bme68x_get_op_mode(&current_op_mode, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_op_mode");

                    rslt_api = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_data");
                    for(uint8_t data_idx = 0; data_idx<n_fields; data_idx++){
                        if(data[data_idx].status & BME68X_GASM_VALID_MSK){
                            uint8_t n_input = 0;
                            bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
                            n_input = processData(currTimeNs, data[data_idx], inputs);
                            if(n_input > 0){
                                uint8_t n_output = REQUESTED_OUTPUT;
                                bsec_output_t output[BSEC_NUMBER_OUTPUTS];
                                memset(output, 0, sizeof(output));
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
            }
        }
        
        /*sleep_run_from_xosc();
        rtc_sleep(3, 0, 0);*/
    }
    save_state_file();
    return 0;
}

//utility to print results
void print_results(int id, float signal, int accuracy){
    switch(id){
        case BSEC_OUTPUT_IAQ:
            printf("IAQ ");
            printf("%.2f  %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_STATIC_IAQ:
            printf("STATIC IAQ ");
            printf("%.2f   %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            printf("CO2[ppm] ");
            printf("%.2f   %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            printf("VOC ");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RAW_TEMPERATURE:
                printf("Temperature[°C] ");
                printf("%.2f \n", signal);
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
                printf("Pressure[Pa] ");
                printf("%.2f \n", signal);
            break;
        case BSEC_OUTPUT_RAW_HUMIDITY:
                printf("Humidity[%%rH] ");
                printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RAW_GAS:
            printf("[Ohm] ");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            printf("Stabilization ");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            printf("Run in ");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            printf("Compensated T ");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            printf("Compensated H ");
            printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_GAS_PERCENTAGE:
            printf("Gas[%%] ");
            printf("%.2f %d\n", signal, accuracy);
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_1:
            if(signal * 10000.0f > 0) /* Ensure that there is a valid value xx.xx% */
                {
                    printf("Probability [%%] ");
                    printf("%.2f\n", signal);
                }
                break;
        case BSEC_OUTPUT_GAS_ESTIMATE_2:
            if(signal * 10000.0f > 0) /* Ensure that there is a valid value xx.xx% */
                {
                    printf("~Probability [%%] ");
                    printf("%.2f\n", signal);
                }
                break;
        case BSEC_OUTPUT_GAS_ESTIMATE_3:
        if(signal * 10000.0f > 0) /* Ensure that there is a valid value xx.xx% */
            {
                printf("Probability 3 [%%] ");
                printf("%.2f\n", signal);
            }
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_4:
            if(signal * 10000.0f > 0) /* Ensure that there is a valid value xx.xx% */
            {
                printf("Probability 4[%%] ");
                printf("%.2f\n", signal);
            }
            break;
        case BSEC_OUTPUT_RAW_GAS_INDEX:
            printf("Gas Index ");
            printf("%.2f\n", signal);
        default:
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
