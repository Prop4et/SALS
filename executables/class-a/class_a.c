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

#include "pico/sleep.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "../lib/bme/bme68x/bme68x.h"
#include "../lib/bme/bme_api/bme68x_API.h"
#include "../lib/bme/bsec/bsec_datatypes.h"
#include "../lib/bme/bsec/bsec_interface.h"
//littlefs
#include "pico_hal.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "lora-config.h"
#include "bme-config.h"
#include "hardware/watchdog.h"

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

/* 
    pin configuration for SX1262 radio module
*/
const struct lorawan_sx12xx_settings sx12xx_settings = {
    .spi = {
        .inst = spi1,
        .mosi = 11,
        .miso = 12,
        .sck  = 10,
        .nss = 3
    },
    .reset = 15,
    .busy = 2,
    .dio1 = 20
};

/*
    ABP settings
*/ 
const struct lorawan_abp_settings abp_settings = {
    .device_address = LORAWAN_DEV_ADDR,
    .network_session_key = LORAWAN_NETWORK_SESSION_KEY,
    .app_session_key = LORAWAN_APP_SESSION_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

/*
    variables for receiving lora downlinks (if any)
*/
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

/*
    Structure of the uplink, total of 12 Bytes
*/
struct uplink {
    uint16_t id; 
    int16_t temp; //-273.15 - 90.00 -> -27315 - 9000
    uint16_t hum; //0.00 - 100.00 -> 0 - 10000
    uint16_t press; //840.0 - 1013.25 [hPa] -> 8400 - 10133
    uint16_t AQI; //50.0 - 500.0 -> 500 - 5000
    uint16_t CO2; //600 - 10000 [ppm] -> 600 - 10000
};

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

//settings to load
uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];
uint32_t n_serialized_settings_max = BSEC_MAX_PROPERTY_BLOB_SIZE;
uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer = BSEC_MAX_WORKBUFFER_SIZE;
uint32_t n_serialized_settings = 0;

/*
    FILESYSTEM VARIABLES
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
 * @brief callback called after the interrupt of a rtc sleep, resets the clock and reestablishes the output if needed
 * 
 */
static void sleep_callback(){
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);
    
    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clocks
    clocks_init();
#ifdef DEBUG //no need if no output
    stdio_uart_init();
#endif
}

/**
 * @brief function to go into deep sleep mode, stops the internal clocks and leaves the minimal logic to wake up active
 * 
 * @param secs seconds to sleep
 * @param mins minutes to sleep
 * @param hrs  hours to sleep
 */
static void rtc_sleep(uint8_t secs, uint8_t mins, uint8_t hrs){
    datetime_t t = {
        .year = 2023,
        .month = 1,
        .day = 01,
        .dotw = 1,
        .hour = 00,
        .min = 00,
        .sec = 00
    };

    datetime_t t_alarm = {
        .year = 2023,
        .month = 1,
        .day = 01,
        .dotw = 1,
        .hour = hrs,
        .min = mins,
        .sec = secs
    };
    
    rtc_init();
    rtc_set_datetime(&t);
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);
}

/**
 * @brief popolates the uplink structure with the values read from the sensor
 * 
 * @param pkt the uplink structure
 * @param output array of the values requested from the BSEC library
 * @param len length of the output array
 */
void make_pkt(struct uplink* pkt, bsec_output_t* output, int len);


/**
 * @brief processes and prepares sensor readings for the bsec library 
 * 
 * @param currTimeNs time of readings in ns
 * @param data data field (signal, id and so on) 
 * @param inputs inputs to fill for the bsec library
 * @return uint8_t 
 */
uint8_t processData(int64_t currTimeNs, const struct bme68x_data data, bsec_input_t* inputs);

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

int main( void )
{   
    /*
        Save clock states to retrieve after deep sleep
    */
    scb_orig = scb_hw->scr;
    clock0_orig = clocks_hw->sleep_en0;
    clock1_orig = clocks_hw->sleep_en1;
    //variables holding the time of the sleep in secs, hours and minutes, cannot overflow
    uint8_t secs = 58;
    uint8_t mins = 4;

    /*
        PKT AND CONSTANT VALUES
    */
    struct uplink pkt = {
        .id = DEV_ID,
    };    
    //
    uint64_t time_us;
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

    uint8_t sent_time = INTERVAL;
    uint8_t saved_time = 0;
    uint64_t before_time = 0;
    uint64_t after_time = 0;
        
    // initialize stdio and wait for USB CDC connect
#ifdef DEBUG   
    stdio_init_all();
#endif

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

    //requested outputs
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_ULP;

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

    /*
        INITIALIZATION BSEC LIBRARY
    */
    rslt_bsec = bsec_init();
    check_rslt_bsec(rslt_bsec, "BSEC_INIT");

    if(rslt_fs > 0){
    #ifdef DEBUG
        printf("...resuming the state, read %d bytes\n", rslt_fs);
    #endif
        //set the state if there is one saved
        rslt_bsec = bsec_set_state(serialized_state, n_serialized_state, work_buffer, n_work_buffer_size);
        check_rslt_bsec(rslt_bsec, "BSEC_SET_STATE");
    }    
    /*
        Set configuration is skipped
    */    
    
    // Call bsec_update_subscription() to enable/disable the requested virtual sensors
    rslt_bsec = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);
    check_rslt_bsec( rslt_bsec, "BSEC_UPDATE_SUBSCRIPTION");  

#ifdef DEBUG
    lorawan_debug(true);
    printf("Pico LoRaWAN - lora and bme sensor\n\n");
    printf("Erasing NVM ... ");
    
    if (lorawan_erase_nvm() < 0) {
        printf("failed!!!\n");
    } else {
        printf("success!\n");
    }
#else 
    if (lorawan_erase_nvm() < 0) {
        blink();
    } 
#endif

    // initialize the LoRaWAN stack
#ifdef DEBUG
    printf("Initilizating LoRaWAN ... ");
#endif
    if (lorawan_init_abp(&sx12xx_settings, LORAWAN_REGION, &abp_settings) < 0) {
    #ifdef DEBUG
        printf("Fail, restarting\n");
    #endif
        software_reset();

    }
#ifdef DEBUG
    else {
        printf("Success!\n");
    }
#endif
      
    /*initialize state for temp/hum/press with impossible values*/
    double previous_temp = -400;
    double previous_hum = -400;
    double previous_press = -400;
    uint8_t current_op_mode = 0;
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    lorawan_process_timeout_ms(10000);
    conf_bsec.next_call = 0;
    // loop forever
    save_state_file();
    while (1) {
        uint8_t nFieldsLeft = 0;
        uint64_t currTimeNs = time_us_64()*1000;
        current_op_mode = conf_bsec.op_mode;
        //main loop operations
        if(conf_bsec.next_call >= conf_bsec.next_call){ 
            rslt_bsec = bsec_sensor_control(conf_bsec.next_call, &conf_bsec); //this one is cheating
            //rslt_bsec = bsec_sensor_control(conf_bsec.next_call, &conf_bsec); //this one is cheating
            check_rslt_bsec(rslt_bsec, "BSEC_SENSOR_CONTROL");
            if(rslt_bsec != BSEC_OK)
                continue;
        
            switch(conf_bsec.op_mode){
                case BME68X_FORCED_MODE:
                #ifdef DEBUG
                    printf("--------------Forced Mode--------------\n");
                #endif
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
                    #ifdef DEBUG
                        printf("--------------Parallel Mode--------------\n");
                    #endif
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
                    #ifdef DEBUG
                        printf("--------------Sleep Mode--------------\n");
                    #endif
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
                            lorawan_process();
                            if(rslt_bsec == BSEC_OK){
                            #ifdef DEBUG
                                printf("------------------Results------------------\n");
                                for(uint8_t  i = 0; i < n_output; i++){
                                    print_results(output[i].sensor_id, output[i].signal, output[i].accuracy);
                                }
                                printf("--------------------------------------------\n");
                            #endif
                                before_time = time_us_64();
                                if(sent_time >= INTERVAL){
                                    make_pkt(&pkt, output, 6);
                                    secs = 55;
                                #ifdef DEBUG
                                    printf("\n");
                                    if (lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2) < 0) {
                                        sent_time += 1;
                                        printf("failed!!!\n");
                                    } else {
                                        printf("success!\n");
                                    }
                                #else
                                    if(lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2) >= 0)
                                        sent_time += 1;
                                #endif
                                    sent_time = 0;
                                    if (lorawan_process_timeout_ms(3000) == 0) { //downlink windows for class A of 1 and 2 secs
                                        // check if a downlink message was received
                                        receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
                                    }
                                }else{
                                    sent_time += 1;
                                    printf("Increasing sent time: %u\n", sent_time);
                                }
                            }
                        }
                    }
                    secs = 58;
                    if(saved_time >= SAVE_INTERVAL){
                        save_state_file();
                        saved_time = 0;
                    }
                    saved_time += 1;
                #ifdef DEBUG
                    printf("Increasing saved time %u\n", saved_time);
                    sleep_ms(200);
                #endif
                    after_time = time_us_64();
                    secs = secs - (after_time-before_time)/1000000;
                    //sleep_ms(298000 - (after_time-before_time)/1000);
                    sleep_run_from_xosc();
                    rtc_sleep(secs, 4, 0);
                }
                //CLASS A sensor should never be in PARALLEL MODE
            }
        } 
    #ifdef DEBUG
        else{
            printf("Didn't sleep enough\n");
        }
    #endif
    }

    return 0;
}

/*
    Creates the uplink packet
*/
void make_pkt(struct uplink* pkt, bsec_output_t* output, int len){
    for(int i = 0; i<len; i++){
        switch(output[i].sensor_id){
            case BSEC_OUTPUT_STATIC_IAQ:
                pkt->AQI = output[i].accuracy < 2 ? 0 : (uint16_t)(output[i].signal*10);
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                pkt->CO2 = output[i].accuracy < 2 ? 0 : (uint16_t)output[i].signal;
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                pkt->temp = (int16_t)(output[i].signal*100);
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
                pkt->hum = (uint16_t)(output[i].signal*100);
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                pkt->press = ((uint16_t)output[i].signal % 10 >= 5) ? (uint16_t)(output[i].signal/10)+1 : (uint16_t)(output[i].signal/10);
                break;
        }
    }
    
}

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

/*
    Function to save the file on littlefs. TODO: change if switch to sd card
*/
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

//utility to print results
void print_results(int id, float signal, int accuracy){
    switch(id){
        case BSEC_OUTPUT_IAQ:
            printf("IAQ\n");
            printf("%.2f  %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_STATIC_IAQ:
            printf("STATIC IAQ\n");
            printf("%.2f %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            printf("CO2[ppm]\n");
            printf("%.2f %d \n", signal, accuracy);
            break;
        case BSEC_OUTPUT_RAW_TEMPERATURE:
                printf("Temperature[°C] \n");
                printf("%.2f \n", signal);
            break;
        case BSEC_OUTPUT_RAW_HUMIDITY:
                printf("Humidity[%%rH] \n");
                printf("%.2f\n", signal);
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
                printf("Pressure[Pa] \n");
                printf("%.2f \n", signal);
            break;
        case BSEC_OUTPUT_RAW_GAS:
            printf("[Ohm]\n");
            printf("%.2f\n", signal, accuracy);
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            printf("VOC\n");
            break;
    }
}

void check_fs_error(int rslt, char msg[]){
    if(rslt < 0){
    #ifdef DEBUG
        printf("FS error %s\n", msg);
    #endif
        blink();
    }
}

