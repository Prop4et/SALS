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

//littlefs
#include "pico_hal.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "bme-config.h"
#include "hardware/watchdog.h"

#define REQUESTED_OUTPUT        4
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

//state to save
uint8_t serialized_state[BSEC_MAX_STATE_BLOB_SIZE];
uint32_t n_serialized_state_max = BSEC_MAX_STATE_BLOB_SIZE;
uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
uint8_t work_buffer_state[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer_size = BSEC_MAX_WORKBUFFER_SIZE;
//configuration on shut down
uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];
uint32_t n_serialized_settings_max = BSEC_MAX_PROPERTY_BLOB_SIZE;
uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer = BSEC_MAX_WORKBUFFER_SIZE;
/*
    File System variables
*/
lfs_size_t rslt_fs;

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

/**
 * @brief signals if there was an error manipulating the file
 * 
 * @param rslt result of the operation
 * @param msg message printed out
 */
void check_fs_error(int rslt, char msg[]);

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
        INITIALIZE GPIO PINS
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
    /*
        little FS set up, it mounts a file system on the flash memory to save the state file
        if set to true it formats everything
        if set to false it keeps the files as they were
    */
    if (pico_mount(format) != LFS_ERR_OK) {
    #ifdef DEBUG
        printf("Error mounting FS\n");
    #endif
        blink();
    }
    gpio_put(PIN_FORMAT_OUTPUT, 0);


    //read state to get the previous state and avoid restarting everything
    int state_file = pico_open(state_file_name, LFS_O_CREAT | LFS_O_RDONLY );
    check_fs_error( state_file, "Error opening state file"); 
    
    rslt_fs = pico_read(state_file, serialized_state, BSEC_MAX_WORKBUFFER_SIZE*sizeof(uint8_t));
    check_fs_error(state_file, "Error while reading state file");  

    pico_rewind(state_file);
    check_fs_error(state_file, "Error while rewinding state file");
    pico_unmount();
    sleep_ms(1000);
    gpio_deinit(PIN_FORMAT_INPUT);
    gpio_deinit(PIN_FORMAT_OUTPUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
#ifdef DEBUG
    printf("...initialization BSEC\n");
#endif
    
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_1;
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_2;
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_3;
    requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_4;
    requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    
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

    rslt_bsec = bsec_init();
    check_rslt_bsec(rslt_bsec, "BSEC_INIT");
    /*
        INITIALIZATION BSEC LIBRARY
    */
#ifdef DEBUG
    bsec_version_t v;
    bsec_get_version(&v);
    printf("Version: %d.%d.%d.%d\n", v.major, v.minor, v.major_bugfix, v.minor_bugfix);
#endif
    if(rslt_fs > 0){
    #ifdef DEBUG
        printf("...resuming the state, read %d bytes\n", rslt_fs);
    #endif
        //set the state if there is one saved
        rslt_bsec = bsec_set_state(serialized_state, n_serialized_state, work_buffer, n_work_buffer_size);
        check_rslt_bsec(rslt_bsec, "BSEC_SET_STATE");
    }    
    #ifdef DEBUG
        printf("...loading configuration\n");
    #endif
    const uint8_t bsec_config_selectivity[2285] = {0,0,2,2,189,1,0,0,0,0,0,0,213,8,0,0,52,0,1,0,0,168,19,73,64,49,119,76,0,192,40,72,0,192,40,72,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,254,131,137,87,88,0,9,0,7,240,150,61,0,0,0,0,0,0,0,0,28,124,225,61,52,128,215,63,0,0,160,64,0,0,0,0,0,0,0,0,205,204,12,62,103,213,39,62,230,63,76,192,0,0,0,0,0,0,0,0,145,237,60,191,251,58,64,63,177,80,131,64,0,0,0,0,0,0,0,0,93,254,227,62,54,60,133,191,0,0,64,64,12,0,10,0,0,0,0,0,0,0,0,0,173,6,11,0,0,0,2,17,41,96,190,41,9,140,61,244,44,244,189,24,63,140,61,123,103,21,62,45,211,24,63,41,80,106,189,217,170,127,63,136,20,213,61,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,223,160,3,191,223,160,3,63,0,0,0,0,0,0,0,0,245,87,188,190,75,93,171,191,70,114,62,191,71,207,0,191,34,37,237,190,6,6,204,62,192,4,134,190,72,115,31,63,211,252,129,191,0,0,0,0,113,139,197,189,121,134,107,190,157,165,52,191,157,8,144,62,91,82,146,191,196,2,179,62,129,10,238,62,60,151,107,63,221,50,49,190,0,0,0,0,224,253,146,62,195,154,236,60,100,174,53,191,247,203,61,190,50,119,143,190,178,117,117,62,211,195,201,189,5,228,114,62,102,95,108,190,0,0,0,0,32,144,175,62,166,42,109,63,197,237,247,190,217,160,206,190,209,7,178,190,245,218,153,190,196,147,183,62,66,167,25,191,245,75,42,63,0,0,0,0,40,152,13,63,4,29,184,62,154,159,183,190,251,36,247,189,205,37,26,61,101,33,146,190,86,100,202,61,240,33,44,190,5,126,42,62,0,0,0,0,34,40,27,63,227,3,37,63,144,19,149,62,4,176,40,63,14,38,160,62,54,249,169,190,112,205,157,61,163,59,66,191,81,12,189,62,0,0,0,0,62,218,55,63,3,102,152,63,7,41,44,62,209,221,232,62,206,197,16,63,95,154,6,190,77,134,10,62,131,135,97,191,129,220,123,63,0,0,0,0,12,230,38,190,228,50,89,62,227,224,129,63,130,213,16,63,47,240,65,62,8,113,122,189,245,68,52,63,160,216,62,61,44,195,216,62,0,0,0,0,119,62,249,61,168,222,55,61,236,150,153,61,121,98,213,190,189,191,250,62,138,41,50,191,246,79,8,191,237,55,111,188,95,253,183,190,0,0,0,0,244,92,181,190,19,175,238,189,253,139,147,190,181,1,123,191,59,192,39,63,25,140,116,189,144,21,131,191,212,126,56,62,47,87,127,188,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,50,150,228,62,22,234,137,191,0,0,0,0,0,0,0,0,146,39,24,63,171,76,129,191,0,0,0,0,0,0,0,0,245,244,134,61,205,152,144,191,0,0,0,0,0,0,0,0,240,153,138,63,62,55,171,191,0,0,0,0,0,0,0,0,188,245,230,63,164,54,127,191,0,0,0,0,0,0,0,0,100,168,240,190,145,58,72,63,0,0,0,0,0,0,0,0,97,171,41,63,163,135,203,191,0,0,0,0,0,0,0,0,87,152,141,191,17,163,201,63,0,0,0,0,0,0,0,0,100,91,116,63,107,83,161,191,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,39,93,132,72,157,0,47,76,80,154,4,76,50,87,176,75,80,6,138,73,169,155,117,73,82,255,95,73,112,36,100,72,216,43,137,72,77,50,149,72,0,0,0,0,0,0,0,0,0,0,0,0,242,86,219,71,197,24,209,75,254,54,159,75,139,69,79,75,17,36,3,73,20,186,229,72,57,74,206,72,34,82,149,71,22,201,157,71,119,145,156,71,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,87,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,0,0,0,0,255,98,0,0};
    rslt_bsec = bsec_set_configuration(bsec_config_selectivity, n_serialized_settings_max, work_buffer, n_work_buffer);
    check_rslt_bsec( rslt_bsec, "BSEC_SET_CONFIGURATION");

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
                    printf("Probability [%%] ");
                    printf("%.2f\n", signal);
                break;
        case BSEC_OUTPUT_GAS_ESTIMATE_2:
                    printf("~Probability [%%] ");
                    printf("%.2f\n", signal);
                break;
        case BSEC_OUTPUT_GAS_ESTIMATE_3:
                printf("Probability 3 [%%] ");
                printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_4:
                printf("Probability 4[%%] ");
                printf("%.2f\n", signal);
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
    #ifdef DEBUG
        printf("...Saving the file\n");
    #endif
        //mount the fs 
    if (pico_mount(true) != LFS_ERR_OK) {
    #ifdef DEBUG
        printf("Error mounting FS\n");
    #endif
        blink();
    }
    //open the file in write mode, no create because it should've been created already
    int state_file = pico_open(state_file_name, LFS_O_CREAT | LFS_O_WRONLY);
    check_fs_error(state_file, "Error opening state file write"); 
    //get the state file
    rslt_bsec = bsec_get_state(0, serialized_state, n_serialized_state_max, work_buffer, n_work_buffer_size, &n_serialized_state);
    check_rslt_bsec(rslt_bsec, "BSEC_GET_STATE");
    //write the file and flush
    rslt_fs = pico_write(state_file, serialized_state, BSEC_MAX_STATE_BLOB_SIZE*sizeof(uint8_t));
    check_fs_error(rslt_fs, "Error writing the file");
    pico_fflush(state_file);
    //log the number of bytes written
    int pos = pico_lseek(state_file, 0, LFS_SEEK_CUR);
#ifdef DEBUG
    printf("Written %d byte for file %s\n", pos, state_file);
#endif
    //rewind the pointer to the beginning of the file just to be sure
    rslt_fs = pico_rewind(state_file);
    check_fs_error(state_file, "Error while rewinding state file");
    //close the file
    rslt_fs = pico_close(state_file);
    //unmount the fs
    pico_unmount();
    //turn off the led, system can be shut down 
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_us(5);
}

void check_fs_error(int rslt, char msg[]){
    if(rslt < 0){
    #ifdef DEBUG
        printf("FS error %s\n", msg);
    #endif
        blink();
    }
}