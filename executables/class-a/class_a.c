/**
 * @file class_a.c
 * @author Francesco Biancucci  
 * @brief This file uses the LoRaWAN protocol to send sensor readings to a server
 * @version 0.1
 * @date 2023-05-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifdef DEBUG
    #include <stdio.h>
#endif
#include <string.h>

/*
pico libraries used to handle the deep sleep
*/
#include "pico/sleep.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"

/*
pico libraries used to handle the sensor
*/
#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "../lib/bme/bme68x/bme68x.h"
#include "../lib/bme/bme_api/bme68x_API.h"
#include "../lib/bme/bsec2_4/bsec_datatypes.h"
#include "../lib/bme/bsec2_4/bsec_interface.h"

/*pico libraries used to handle the filesystem*/
#include "pico_hal.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "lora-config.h"
#include "bme-config.h"
#include "hardware/watchdog.h"

//number of requested output from the bsec library
#define REQUESTED_OUTPUT        6
//binary mask to check if the readings are valid
#define BME68X_VALID_DATA       UINT8_C(0xB0)
//macro to check if the raw data is used to determine the derived data in the bsec library
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
    the values obtained from the uplink need to be converted,
    sending them as integers requires multipling them by a factor (in the curly brackets)
    that then has to be used for the opposite conversion process
*/
struct uplink {
    uint16_t id; 
    int16_t temp; //-273.15 - 90.00 -> -27315 - 9000 {100}
    uint16_t hum; //0.00 - 100.00 -> 0 - 10000 {100}
    uint16_t press; //840.0 - 1013.25 [hPa] -> 8400 - 10133 {10, ignore last decimal number}
    uint16_t AQI; //50.0 - 500.0 -> 500 - 5000 {10}
    uint16_t CO2; //600 - 10000 [ppm] -> 600 - 10000
};

//bsec measurement
bsec_sensor_configuration_t requested_virtual_sensors[REQUESTED_OUTPUT];
uint8_t n_requested_virtual_sensors = REQUESTED_OUTPUT;
/*
    Allocate a struct for the returned physical sensor settings 
    retunred physical sensors are the values requested to the bsec library, it has nothing to
    do with the number of bme688 sensors installed on the MCU
*/
bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR]; 
uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
//configuration coming from bsec
bsec_bme_settings_t conf_bsec;

/*
    variables used to hold the state for the bsec library
    state should be saved pretty often, so in case the MCU
    shuts down it can be restored
*/
uint8_t serialized_state[BSEC_MAX_STATE_BLOB_SIZE];
uint32_t n_serialized_state_max = BSEC_MAX_STATE_BLOB_SIZE;
uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
uint8_t work_buffer_state[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer_size = BSEC_MAX_WORKBUFFER_SIZE;

/*
    for class a no settings are required to load
    since nothing works on prefigured/pretrained values
*/

/*
    FILESYSTEM OPERATION RESULTS
*/
lfs_size_t rslt_fs;
/*
    BSEC OPERATION RESULTS
*/
bsec_library_return_t rslt_bsec;

/**
 * @brief watchdog that goes on a loop to force a reset of the pico
 * in case of errors
 */
void software_reset()
{
    watchdog_enable(1, 1);
    while(1);
}

/**
 * @brief callback called after the interrupt of a rtc sleep, 
 * resets the clock and re-establishes the output if needed
 */
static void sleep_callback(){

    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);
    
    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clocks
    clocks_init();
    stdio_uart_init();
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
    
    //initializes the real time clock
    rtc_init();
    //sets a default datetime to the real time clock
    rtc_set_datetime(&t);
    //sets the future time with respect to the datetime set for waking up
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
 * @return uint8_t the number of raw inputs passed to the bsec library
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
 * @brief signals if there was an error related to the filesystem operations
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
    /*
        BME API VARIABLES
        bme holds the physical info of the sensor
        data holds all the info obtained from the sensor when reading the data
        conf holds the configuration parameter for the sensor
        heatr_conf holds the configuration parameter for the heater plate used when doing gas sensors
    */
    struct bme68x_dev bme;
    struct bme68x_data data[3];
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;

    /*
        rlst_api holds the result of bsec api operation
        n_fields is used by get data and holds the number of data instances available 
    */
    int8_t rslt_api;
    uint8_t n_fields;

    /*
        del_persiod is the amount of time to wait before reading to heat the plate
        sent_time is the counter for the number of time that a reading has been made but not sent
        saved_time is the counter for the number of time that a reading has been made but the state is not saved
        before_time e after_time are two variables used to compute the amount of time elapsed between a reading and all the other operations
        this time is then used to scale the sleep time correctly
    */
    uint32_t del_period;
    uint16_t sent_time = INTERVAL;
    uint8_t saved_time = SAVE_INTERVAL;
    uint64_t before_time = 0;
    uint64_t after_time = 0;
        
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();

    /*
        INITIALIZE GPIO PINS
        initializes led pin, blinks if something happens and is turned on when doing fs operations
        initializes gpio pins used to determine if the fs needs to be formatted or not
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
    //read the pin state 
    bool format = gpio_get(PIN_FORMAT_INPUT);
#ifdef DEBUG
    printf("...mounting FS");
    format ? printf(" and formatting\n") : printf("\n");
#endif
    /*
        little FS set up, it mounts a file system on the flash memory to save the state file
        if set to true it formats everything
        if set to false it keeps the files as they are and resumes the state
        FORMAT IF THE NODE IS MOVED FROM ONE LOCATION TO ANOTHER ONE AND IF IT IS THE FIRST
        TIME THE NODE IS TURNED ON
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
    
    rslt_fs = pico_read(state_file, serialized_state, BSEC_MAX_STATE_BLOB_SIZE*sizeof(uint8_t));
    check_fs_error(state_file, "Error while reading state file");  

    pico_rewind(state_file);
    check_fs_error(state_file, "Error while rewinding state file");
    //unmount operation is always executed, so in case of sudden shut down data isn't lost
    pico_unmount();
    sleep_ms(1000);
    //deinit pins, they are no longer used until the device is restarted
    gpio_deinit(PIN_FORMAT_INPUT);
    gpio_deinit(PIN_FORMAT_OUTPUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

#ifdef DEBUG
    printf("...initialization BSEC\n");
#endif

    //requested outputs in ULP mode, readings happen in an interval of 5 minutes
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ; //index air quality
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE; //temperature
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE; //pressure
    requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY; //humidity
    requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT; //CO2 in ppm
    requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_GAS; //gas resistance
    requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_ULP;

    /*
        INITIALIZATION BME CONFIGURATION
    */
#ifdef DEBUG
    printf("...initialization BME688\n");
#endif
    bme_interface_init(&bme, BME68X_I2C_INTF);
    /*
        variable holding the identifier of the sensor
    */
    uint8_t data_id;

    /*
        read device id after init to see if the comms work properly and the sensor is recognized
    */
    bme.read(BME68X_REG_CHIP_ID, &data_id, 1, bme.intf_ptr);
    if(data_id != BME68X_CHIP_ID){
    #ifdef DEBUG
        printf("Cannot communicate with BME688\n");
        printf("CHIP_ID: %x \t ID_READ: %x\n", BME68X_CHIP_ID, data_id);
    #endif
        blink();
    }
    else{
        //save device id in the structure
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

    /*
        reading from the file returned bytes, so there is a state file saved
    */
    if(rslt_fs > 0){
    #ifdef DEBUG
        printf("...resuming the state, read %d bytes\n", rslt_fs);
    #endif
        //set the state if there is one saved
        rslt_bsec = bsec_set_state(serialized_state, n_serialized_state, work_buffer_state, n_work_buffer_size);
        check_rslt_bsec(rslt_bsec, "BSEC_SET_STATE");
    }    
    /*
        Set configuration is skipped for class a
    */    
    
    // Call bsec_update_subscription() to enable/disable the requested virtual sensors
    rslt_bsec = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);
    check_rslt_bsec( rslt_bsec, "BSEC_UPDATE_SUBSCRIPTION");  

    //if debug sets lorawan debug messages to be printed

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
      
    /*initialize state for temp/hum/press with unaccepted values*/
    double previous_temp = -400;
    double previous_hum = -400;
    double previous_press = -400;
    uint8_t current_op_mode = BME68X_SLEEP_MODE;
    uint16_t current_interval = INTERVAL;

    /*
        using abp it is a pass through function
        but it stays in case of switch to otaa when using a proper 
        gateway
    */
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    lorawan_process_timeout_ms(1000);

    // loop forever
    while (1) {
        current_op_mode = conf_bsec.op_mode;
        /*
            cheats the bsec sensor control timer
            using deep sleep stops the internal clock so the amount of time waiting in sleep cannot be measured
            with the clocks
        */        
        rslt_bsec = bsec_sensor_control(conf_bsec.next_call, &conf_bsec); 
        check_rslt_bsec(rslt_bsec, "BSEC_SENSOR_CONTROL");
        if(rslt_bsec != BSEC_OK)
            continue;
        if(conf_bsec.op_mode != current_op_mode){
            /*
                checks which operation mode needs to be used
            */
            switch(conf_bsec.op_mode){
                case BME68X_FORCED_MODE:
                    #ifdef DEBUG
                        printf("--------------Forced Mode--------------\n");
                    #endif
                        /*
                            sets the configuration for the sensor
                        */
                        conf.filter = BME68X_FILTER_OFF;
                        conf.odr = BME68X_ODR_NONE;
                        conf.os_hum = conf_bsec.humidity_oversampling;
                        conf.os_pres = conf_bsec.pressure_oversampling;
                        conf.os_temp = conf_bsec.temperature_oversampling;
                        rslt_api = bme68x_set_conf(&conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_conf");

                        /*
                            sets the configuration for the heater
                        */
                        heatr_conf.enable = BME68X_ENABLE;
                        heatr_conf.heatr_temp = conf_bsec.heater_temperature;
                        heatr_conf.heatr_dur = conf_bsec.heater_duration;
                        rslt_api = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_heatr_conf");

                        current_op_mode = BME68X_FORCED_MODE;
                    break;
                case BME68X_PARALLEL_MODE:
                    #ifdef DEBUG
                        printf("--------------Parallel Mode--------------\n");
                    #endif
                        /*
                            parallel mode is ignored because the ULP sample mode means the sensor uses forced mode
                        */
                    break;
                case BME68X_SLEEP_MODE:
                    if (current_op_mode != conf_bsec.op_mode){
                        /*
                            after a reading the sensor is automatically put in sleep mode if the reading happens in forced mode
                            from the BOSCH documentation
                        */ 
                    #ifdef DEBUG
                        printf("--------------Sleep Mode--------------\n");
                    #endif
                        rslt_api = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme); 
                        current_op_mode = BME68X_SLEEP_MODE;
                    }
                    break;
            }
        }
        
        /*
            checks if a measurement is requested, if so do it
        */
        if(conf_bsec.trigger_measurement){
            if(conf_bsec.op_mode == BME68X_FORCED_MODE){
                rslt_api = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
                check_rslt_api(rslt_api, "bme68x_set_op_mode");

                del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
                bme.delay_us(del_period, bme.intf_ptr);
                
                //get raw data
                rslt_api = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &bme);
                check_rslt_api(rslt_api, "bme68x_get_data");
                /*
                    in forced mode only data[0] is written, if the readings are valid proceed to pass them to the library
                */
                if(data[0].status & BME68X_GASM_VALID_MSK){
                    uint8_t n_input = 0;
                    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
                    //prepare the inputs for the bsec library
                    n_input = processData(conf_bsec.next_call, data[0], inputs);
                    if(n_input > 0){
                        //prepare the array for the output from the bsec library
                        uint8_t n_output = REQUESTED_OUTPUT;
                        bsec_output_t output[BSEC_NUMBER_OUTPUTS];
                        memset(output, 0, sizeof(output));
                        
                        //call library function
                        rslt_bsec = bsec_do_steps(inputs, n_input, output, &n_output);
                        if(rslt_bsec == BSEC_OK){
                        #ifdef DEBUG
                            printf("------------------Results------------------\n");
                            for(uint8_t  i = 0; i < n_output; i++){
                                print_results(output[i].sensor_id, output[i].signal, output[i].accuracy);
                            }
                            printf("--------------------------------------------\n");
                        #endif
                            /*
                                once all the operations from the library are done save the time for the operation required for the LoRaWAN stack
                                if it's time to send out a packet send it
                            */
                            before_time = time_us_64();
                            secs = 58;
                            if(sent_time >= current_interval){
                                make_pkt(&pkt, output, REQUESTED_OUTPUT);
                            #ifdef DEBUG
                                printf("\n");
                                if (lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2) < 0) {
                                    printf("failed!!!\n");
                                }else{ 
                                    printf("success!\n");
                                }
                            #else
                                lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2);
                            #endif
                                sent_time = 1;
                                #ifdef DEBUG
                                    printf("Resetting sent time: %u\n", sent_time);
                                #endif
                                /*
                                    process LoRaWAN events, give time to the irq do go down before deep sleep, otherwise it bugs and 
                                    the next time it tries to send the irq results busy, then check for eventual downlinks
                                */
                                lorawan_process_timeout_ms(4500);
                                receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
                                if(receive_length == 2){
                                    current_interval = receive_buffer[1] + (receive_buffer[0] << 8);
                                #ifdef DEBUG
                                    printf("Buffer received: least significant %u most significant %u", receive_buffer[1], receive_buffer[0]);
                                    printf("New interval time: %d", current_interval);
                                #endif
                                }
                            #ifdef DEBUG
                                /*
                                    handle interval time as downlink to change it
                                */
                                if(receive_length == 2){
                                    current_interval = receive_buffer[1] + (receive_buffer[0] << 8);
                                #ifdef DEBUG
                                    printf("Buffer received: least significant %u most significant %u", receive_buffer[1], receive_buffer[0]);
                                    printf("New interval time: %d", current_interval);
                                #endif
                                }
                                /*
                                    prepare for handling eventual different downlinks
                                */
                                else if (receive_length > -1) {
                                #ifdef DEBUG
                                    printf("received a %d byte message on port %d: ", receive_length, receive_port);
                                #endif
                                    for (int i = 0; i < receive_length; i++) {
                                        printf("%02x", receive_buffer[i]);
                                    }
                                    printf("\n");
                                }
                            #endif
                                receive_length = 0;
                            }else{
                                sent_time += 1;
                            #ifdef DEBUG
                                printf("Increasing sent time: %u\n", sent_time);
                            #endif

                            }
                        }
                    }
                }
                //check if the time has come to save the state file
                if(saved_time >= SAVE_INTERVAL){
                    save_state_file();
                    saved_time = 1;
                    #ifdef DEBUG
                        printf("Resetting saved time %u\n", saved_time);
                        sleep_ms(200);
                    #endif
                }else{
                    saved_time += 1;
                #ifdef DEBUG
                    printf("Increasing saved time %u\n", saved_time);
                    sleep_ms(200);
                #endif
                }
                /*
                    get the time after all the operations are completed, compute the amount of seconds that the MCU will go to sleep,
                    baseline for the minutes is 4 since from the BOSCH documentation the standard amount of sleep time for the ULP
                    sample rate is 4 minutes and 58 seconds
                */
                after_time = time_us_64();
                secs = secs - (after_time-before_time)/1000000;
                sleep_run_from_xosc();
                rtc_sleep(secs, 4, 0);
            }
        }
    }

    return 0;
}

/**
 * @brief fills the fields for the uplink
 * 
 * @param pkt actual uplink packet
 * @param output values obtained from the bsec library
 * @param len length of the output array
 */
void make_pkt(struct uplink* pkt, bsec_output_t* output, int len){
    for(int i = 0; i<len; i++){
        /*
            BSEC_OUTPUT_STATIC_IAQ if the accuracy is above 1 (sensor is still calibrating but readings have meanings) take the value*10 to consider the first decimal, otherwise 0
            BSEC_OUTPUT_CO2_EQUIVALENT accuracy as above, gets the signal as it is without considering decimal precision since doens't matter that much
            BSEC_OUTPUT_RAW_TEMPERATURE get the temperature value*100 to consider the first two decimals
            BSEC_OUTPUT_RAW_HUMIDITY get the humidity value*100 to consider the first two decimals
            BSEC_OUTPUT_RAW_PRESSURE pressure values are in [Pa] but [hPa] makes more sense, check if there's the need to round by excess, then take the [Pa] value without the last digit and rounded
                                     the actual value that is sent out is in the form of daPa
        */
        switch(output[i].sensor_id){
            case BSEC_OUTPUT_IAQ:
                /*pkt->AQI = output[i].accuracy < 2 ? 0 : (uint16_t)(output[i].signal*10);*/
                pkt->AQI = (uint16_t)(output[i].signal*10);
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                /*pkt->CO2 = output[i].accuracy < 2 ? 0 : (uint16_t)output[i].signal;*/
                pkt->CO2 = (uint16_t)output[i].signal;
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
    /* 
        Checks all the required sensor inputs, required for the BSEC library for the requested outputs 
        the & operation controls if the value is required to be processed
    */
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
    /*
        for the gas values other than checking if it is needed it also needs to be valid
    */
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

/**
 * @brief save the state file on the filesystem
 * 
 */
void save_state_file(){
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    #ifdef DEBUG
        printf("...Saving the file\n");
    #endif
        //mount the fs 
    if (pico_mount(true) != LFS_ERR_OK) {
    #ifdef DEBUG
        printf("Error mounting FS\n");
    #endif
        return;
    }
    //open the file in write mode, no create because it should've been created already
    int state_file = pico_open(state_file_name, LFS_O_CREAT | LFS_O_WRONLY);
    //check_fs_error(state_file, "Error opening state file write"); 
    if(state_file < 0){
        pico_unmount();
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        return;
    }
    //get the state file
    rslt_bsec = bsec_get_state(0, serialized_state, n_serialized_state_max, work_buffer_state, n_work_buffer_size, &n_serialized_state);
    check_rslt_bsec(rslt_bsec, "BSEC_GET_STATE");
    //write the file and flush
    rslt_fs = pico_write(state_file, serialized_state, BSEC_MAX_STATE_BLOB_SIZE*sizeof(uint8_t));
    //check_fs_error(rslt_fs, "Error writing the file");
    if(rslt_fs < 0){
        pico_unmount();
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        return;
    }
    pico_fflush(state_file);
    //log the number of bytes written
    int pos = pico_lseek(state_file, 0, LFS_SEEK_CUR);
#ifdef DEBUG
    printf("Written %d byte for file %s\n", pos, state_file);
#endif
    //rewind the pointer to the beginning of the file just to be sure
    rslt_fs = pico_rewind(state_file);
    if(rslt_fs < 0){
        pico_unmount();
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        return;
    }
    //check_fs_error(state_file, "Error while rewinding state file");
    //close the file
    rslt_fs = pico_close(state_file);
    if(rslt_fs < 0){
        pico_unmount();
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        return;
    }
    //unmount the fs
    pico_unmount();
    //turn off the led, system can be shut down 
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_us(5);
}

//utility function to print out results and accuracy if if matters
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

/*
    utility function to check the operation on the filesystem
*/
void check_fs_error(int rslt, char msg[]){
    if(rslt < 0){
    #ifdef DEBUG
        printf("FS error %s\n", msg);
    #endif
        blink();
    }
}

