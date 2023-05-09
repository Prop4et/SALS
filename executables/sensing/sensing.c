/**
 * @file sensing.c
 * @author Francesco Biancucci
 * @brief small example to check the different operational modes of the bme sensor
 *          and to verify the correct process for the sensing operation of the bsec library
 * @version 0.1
 * @date 2023-05-09
 * 
 * 
 */

#ifdef DEBUG
    #include <stdio.h>
#endif
#include <string.h>


#include "pico/stdlib.h"
#include "../lib/bme/bme68x/bme68x.h"
#include "../lib/bme/bme_api/bme68x_API.h"
#include "../lib/bme/bsec2_0/bsec_datatypes.h"
#include "../lib/bme/bsec2_0/bsec_interface.h"
#include "pico/sleep.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"
//littlefs
#include "pico_hal.h"

#include "hardware/watchdog.h"

#define REQUESTED_OUTPUT        7
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
bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR]; 
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
 * @param rslt_api result of the operation
 * @param msg message printed out
 */
void check_fs_error(int rslt_api, char msg[]);

uint8_t processData(int64_t currTimeNs, const struct bme68x_data d, bsec_input_t* inputs){
    uint8_t n_input = 0;
    
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (conf_bsec.process_data & BSEC_PROCESS_TEMPERATURE)
    {
        inputs[n_input].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[n_input].signal = 0;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
        
        inputs[n_input].signal = d.temperature;
        inputs[n_input].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if (conf_bsec.process_data & BSEC_PROCESS_HUMIDITY)
    {
        inputs[n_input].signal = d.humidity;
        inputs[n_input].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if (conf_bsec.process_data & BSEC_PROCESS_PRESSURE)
    {
        inputs[n_input].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[n_input].signal = d.pressure;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if ((conf_bsec.process_data & BSEC_PROCESS_GAS) && (d.status & BME68X_GASM_VALID_MSK))
    {
        inputs[n_input].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[n_input].signal = d.gas_resistance;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }
    if (BSEC_CHECK_INPUT(conf_bsec.process_data, BSEC_INPUT_PROFILE_PART) && (d.status & BME68X_GASM_VALID_MSK)){
        inputs[n_input].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[n_input].signal = (conf_bsec.op_mode == BME68X_FORCED_MODE) ? 0 : d.gas_index;
        inputs[n_input].time_stamp = currTimeNs;
        n_input++;
    }

    return n_input;
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


const char gasName[4][12] = { "Clean Air", "Coffee", "Undefined 3", "Undefined 4"};

int main( void )
{   
    /*
        Save clock states to retrieve after deep sleep
    */
    scb_orig = scb_hw->scr;
    clock0_orig = clocks_hw->sleep_en0;
    clock1_orig = clocks_hw->sleep_en1;
    //variables holding the time of the sleep in secs, hours and minutes, cannot overflow
    uint8_t secs = 57;
    uint8_t mins = 4;

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
    
    // initialize stdio and wait for USB CDC connect
    stdio_uart_init();
    sleep_ms(5000);
    /*
        INITIALIZATION BME CONFIGURATION
    */
#ifdef DEBUG
    printf("...initialization BME688\n");
#endif
    rslt_api = bme_interface_init(&bme, BME68X_I2C_INTF);
    check_rslt_api(rslt_api, "bme68x_set_conf");

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
    /*Initialize bme688 sensor*/
    rslt_api = bme68x_init(&bme);
    check_rslt_api(rslt_api, "bme68x_init");
    /*Initialize bsec library*/
    rslt_bsec = bsec_init();
    check_rslt_bsec(rslt_bsec, "bsec_init");
    bsec_version_t v;
    bsec_get_version(&v);
    printf("Version: %d.%d.%d.%d\n", v.major, v.minor, v.major_bugfix, v.minor_bugfix);

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
    check_fs_error(state_file, "Error opening state file"); 
    
    rslt_fs = pico_read(state_file, serialized_state, BSEC_MAX_STATE_BLOB_SIZE*sizeof(uint8_t));
    check_fs_error(state_file, "Error while reading state file");  

    pico_rewind(state_file);
    check_fs_error(state_file, "Error while rewinding state file");
    pico_unmount();
    sleep_ms(1000);
    gpio_deinit(PIN_FORMAT_INPUT);
    gpio_deinit(PIN_FORMAT_OUTPUT);
    if(rslt_fs > 0){
    #ifdef DEBUG
        printf("...resuming the state, read %d bytes\n", rslt_fs);
    #endif
        //set the state if there is one saved
        rslt_bsec = bsec_set_state(serialized_state, n_serialized_state, work_buffer, n_work_buffer_size);
        check_rslt_bsec(rslt_bsec, "BSEC_SET_STATE");
    }
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    
#ifdef DEBUG
    printf("...initialization BSEC\n");
#endif

    /*Set configuration and state but it's optional so for now we leave it commented out*/
    const uint8_t bsec_config_selectivity[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0,0,2,2,189,1,0,0,0,0,0,0,213,8,0,0,52,0,1,0,0,168,19,73,64,49,119,76,0,192,40,72,0,192,40,72,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,254,131,137,87,88,0,9,0,7,240,150,61,0,0,0,0,0,0,0,0,28,124,225,61,52,128,215,63,0,0,160,64,0,0,0,0,0,0,0,0,205,204,12,62,103,213,39,62,230,63,76,192,0,0,0,0,0,0,0,0,145,237,60,191,251,58,64,63,177,80,131,64,0,0,0,0,0,0,0,0,93,254,227,62,54,60,133,191,0,0,64,64,12,0,10,0,0,0,0,0,0,0,0,0,173,6,11,0,0,0,2,97,212,217,189,123,211,184,190,246,39,132,190,206,174,109,189,251,75,175,189,235,9,110,62,137,144,36,63,45,8,80,62,144,77,210,188,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,255,226,62,40,255,226,190,0,0,0,0,0,0,0,0,76,31,165,190,5,133,25,190,99,111,16,191,4,102,151,189,223,240,98,190,35,221,96,62,233,47,232,61,154,195,212,62,246,23,39,191,0,0,0,0,208,204,147,189,31,212,43,190,235,102,187,62,96,223,37,190,68,35,41,190,176,189,140,62,167,195,139,189,61,247,59,62,197,184,64,62,0,0,0,0,244,158,240,189,150,236,38,62,220,212,82,190,97,85,116,190,38,131,133,189,226,168,44,62,210,144,202,190,155,4,251,62,111,28,141,62,0,0,0,0,11,238,37,61,214,142,233,189,152,81,180,190,225,50,209,62,51,229,221,62,153,207,193,59,0,126,171,60,100,47,212,62,12,59,73,189,0,0,0,0,109,51,81,189,246,41,221,189,14,235,164,190,106,152,64,62,146,87,64,62,211,57,245,189,85,105,18,61,201,169,91,190,254,132,14,189,0,0,0,0,67,219,100,62,66,204,199,190,41,243,253,189,179,13,234,189,8,59,224,190,29,6,33,190,164,176,176,190,54,130,42,63,55,59,158,189,0,0,0,0,180,113,104,190,83,10,224,190,121,202,43,190,103,45,12,190,15,201,28,190,45,147,66,63,59,77,166,189,87,205,216,189,202,231,80,190,0,0,0,0,61,78,135,190,204,10,107,190,83,139,36,62,193,61,191,62,98,160,17,190,189,93,7,63,134,130,186,61,225,40,223,189,104,13,99,190,0,0,0,0,255,48,206,190,218,86,40,189,67,21,240,190,140,32,28,61,216,22,56,190,200,133,35,190,235,148,37,62,54,40,19,63,59,144,196,190,0,0,0,0,56,41,1,191,129,1,168,190,155,197,38,190,19,130,161,190,172,193,237,189,76,39,22,62,156,12,115,63,153,230,241,59,251,43,182,190,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,159,49,7,191,186,149,18,63,0,0,0,0,0,0,0,0,24,104,156,190,241,167,235,189,0,0,0,0,0,0,0,0,42,27,1,190,234,50,155,62,0,0,0,0,0,0,0,0,104,247,151,189,48,189,192,62,0,0,0,0,0,0,0,0,223,179,175,190,87,168,137,190,0,0,0,0,0,0,0,0,44,240,99,62,155,142,95,191,0,0,0,0,0,0,0,0,74,14,53,63,152,160,21,191,0,0,0,0,0,0,0,0,115,135,204,62,33,73,249,190,0,0,0,0,0,0,0,0,138,35,98,191,36,48,73,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,88,154,50,72,197,13,232,75,9,255,180,75,145,98,131,75,167,32,87,73,174,158,62,73,8,35,45,73,12,163,26,72,242,227,55,72,126,235,71,72,0,0,0,0,0,0,0,0,0,0,0,0,184,21,18,72,175,32,249,75,207,100,195,75,164,64,141,75,117,176,79,73,115,35,54,73,215,19,36,73,36,167,240,71,157,166,10,72,81,152,20,72,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,87,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,0,0,0,0,217,86,0,0};
    rslt_bsec = bsec_set_configuration(bsec_config_selectivity, n_serialized_settings_max, work_buffer, n_work_buffer);
    check_rslt_bsec( rslt_bsec, "BSEC_SET_CONFIGURATION");
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_ULP;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[6].sample_rate = BSEC_SAMPLE_RATE_ULP;
    
    rslt_bsec = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);
    check_rslt_bsec( rslt_bsec, "BSEC_UPDATE_SUBSCRIPTION");
    uint8_t current_op_mode = BME68X_SLEEP_MODE;
    uint32_t sleep_time = 0;
    uint64_t start_time = time_us_64();
    
    //loop
    while (1){
        current_op_mode = conf_bsec.op_mode;
        uint64_t currTimeNs = time_us_64()*1000;
        //if(currTimeNs >= conf_bsec.next_call){
            rslt_bsec = bsec_sensor_control(conf_bsec.next_call, &conf_bsec);
            check_rslt_bsec(rslt_bsec, "BSEC_SENSOR_CONTROL");
            if(rslt_bsec != BSEC_OK)
                continue;
            if(conf_bsec.op_mode != current_op_mode){
                switch(conf_bsec.op_mode){
                    case BME68X_FORCED_MODE:
                        printf("-----------Forced Mode Setup-----------\n");
                        conf.filter = BME68X_FILTER_OFF;
                        conf.odr = BME68X_ODR_NONE;
                        conf.os_hum = conf_bsec.humidity_oversampling;
                        conf.os_pres = conf_bsec.pressure_oversampling;
                        conf.os_temp = conf_bsec.temperature_oversampling;
                        rslt_api = bme68x_set_conf(&conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_conf");

                        /* Check if rslt_api == BME68X_OK, report or handle if otherwise */
                        heatr_conf.enable = BME68X_ENABLE;
                        heatr_conf.heatr_temp = conf_bsec.heater_temperature;
                        heatr_conf.heatr_dur = conf_bsec.heater_duration;
                        rslt_api = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_heatr_conf");
                        
                        current_op_mode = BME68X_FORCED_MODE;
                    break;
                    case BME68X_PARALLEL_MODE:
                        printf("-----------Parallel Mode Setup-----------\n");
                        conf.filter = BME68X_FILTER_OFF;
                        conf.odr = BME68X_ODR_NONE;
                        conf.os_hum = conf_bsec.humidity_oversampling;
                        conf.os_pres = conf_bsec.pressure_oversampling;
                        conf.os_temp = conf_bsec.temperature_oversampling;
                        rslt_api = bme68x_set_conf(&conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_conf");

                        /* Check if rslt_api == BME68X_OK, report or handle if otherwise */
                        heatr_conf.enable = BME68X_ENABLE;
                        heatr_conf.heatr_temp_prof = conf_bsec.heater_temperature_profile;
                        heatr_conf.heatr_dur_prof = conf_bsec.heater_duration_profile;
                        heatr_conf.profile_len = conf_bsec.heater_profile_len;
                        heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000);
                        rslt_api = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_heatr_conf");
                        rslt_api = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_op_mode");
                        current_op_mode = BME68X_PARALLEL_MODE;
                    break;
                    case BME68X_SLEEP_MODE:
                        if (current_op_mode != conf_bsec.op_mode){
                            printf("--------------Sleep Mode--------------\n");
                            rslt_api = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme); 
                            current_op_mode = BME68X_SLEEP_MODE;
                        }
                    break;
                }
            }

            if(conf_bsec.trigger_measurement){
                /* Calculate delay period in microseconds */
                switch(conf_bsec.op_mode){
                    case BME68X_FORCED_MODE:
                        printf("-----------Forced Mode Readings-----------\n");
                        rslt_api = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
                        check_rslt_api(rslt_api, "bme68x_set_op_mode");
                        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
                        bme.delay_us(del_period, bme.intf_ptr);

                        /* Check if rslt_api == BME68X_OK, report or handle if otherwise */
                        rslt_api = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &bme);
                        check_rslt_api(rslt_api, "bme68x_get_data");
                        if(data[0].status & BME68X_GASM_VALID_MSK){
                            uint8_t n_input = 0;
                            bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
                            n_input = processData(conf_bsec.next_call, data[0], inputs);
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
                                    sleep_ms(1000);
                                }
                            }
                        }   
                    break;
                    case BME68X_PARALLEL_MODE:
                        //printf("-----------Parallel Mode Readings-----------\n");
                        del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);
                        bme.delay_us(del_period, bme.intf_ptr);
                        /* Check if rslt_api == BME68X_OK, report or handle if otherwise */
                        rslt_api = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, &bme);
                        check_rslt_api(rslt_api, "bme68x_get_data");
                        for(uint8_t data_idx = 0; data_idx<n_fields; data_idx++){
                            uint8_t n_input = 0;
                            bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
                            //printf("Gas %f 0x%x\n", data[data_idx].gas_resistance, data[data_idx].status);
                            n_input = processData(conf_bsec.next_call, data[data_idx], inputs);
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
                    break;
                }
                last_saved++;
                sleep_run_from_xosc();
                rtc_sleep(57, 4, 0);
            }else{
                printf("No trigger why tho\n");
            }
        //}  

        if(last_saved == 12 && conf_bsec.op_mode == BME68X_SLEEP_MODE){
            printf("Saving\n");
            save_state_file();
            start_time = time_us_64();
            last_saved = 0;
        }
    }


    
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
        case BSEC_OUTPUT_GAS_ESTIMATE_2:
        case BSEC_OUTPUT_GAS_ESTIMATE_3:
        case BSEC_OUTPUT_GAS_ESTIMATE_4:
                if(signal * 10000.0f > 0){
                    printf("%s probability: %.2f%% ", gasName[(id - BSEC_OUTPUT_GAS_ESTIMATE_1)], signal*100);
                    printf("gas accuracy = %d\n", accuracy);
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