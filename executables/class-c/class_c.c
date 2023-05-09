/**
 * @file class_c.c
 * @author Francesco Biancucci
 * @brief uses class c to go in high rate sampling mode (continuous) and check if the presence of some gas is/environmental situation is recognized
 *          the process works fine, the recognition is wrong even if different configuration for the training process are used. The problem seems to be that
 *          the BOSCH devboard used to get the readings that then generate the configuration seems to generate something that is too strictly correlated to the
 *          device used for the configuration generation. The project is then kept in a stall state 
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
#include "pico/lorawan.h"
#include "../lib/bme/bme68x/bme68x.h"
#include "../lib/bme/bme_api/bme68x_API.h"
#include "../lib/bme/bsec2_4/bsec_datatypes.h"
#include "../lib/bme/bsec2_4/bsec_interface.h"

//littlefs
#include "pico_hal.h"

#include "sd_card.h"
#include "ff.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "lora-config.h"
#include "bme-config.h"
#include "hardware/watchdog.h"

//only gas estimates 1 and gas estimates 2 considered
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
    Structure of the uplink, total of 10 Bytes
    the structure holds the probability for the different gases
*/
struct uplink {
    uint16_t id; 
    uint16_t p1;
    uint16_t p2;
    uint16_t p3;
    uint16_t p4;
};

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

//settings to load
uint8_t serialized_settings[BSEC_MAX_PROPERTY_BLOB_SIZE];
uint32_t n_serialized_settings_max = BSEC_MAX_PROPERTY_BLOB_SIZE;
uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
uint32_t n_work_buffer = BSEC_MAX_WORKBUFFER_SIZE;

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
 * @brief adds the probabilites of the different gases obtained throught the reading cycles
 * 
 * @param pkt uplink packet
 * @param id id of the probability 
 * @param signal value of the probability
 */
void add_probabilites(struct uplink* pkt, int id, float signal);

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

int main( void )
{   
    
    uint8_t not_sent_loops = 0;
    uint32_t last_saved = 0;

    /*
        PKT AND CONSTANT VALUES
    */
    struct uplink pkt = {
        .id = DEV_ID,
        .p1 = 0,
        .p2 = 0,
        .p3 = 0,
        .p4 = 0,
    };   
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
    /*
        load the configuration with the parameters for the gas recognition, the configuration is obtained through the bosch bme ai sensor software
        there's the chance to load the figuration as a file but it doesn't work, it was nontheless used through a sd card reader connected to the pico
    */
    const uint8_t bsec_config_selectivity[1974] = {0,0,4,2,189,1,0,0,0,0,0,0,158,7,0,0,176,0,1,0,0,168,19,73,64,49,119,76,0,192,40,72,0,192,40,72,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,0,0,96,64,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,205,204,204,189,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,128,63,82,73,157,188,95,41,203,61,118,224,108,63,155,230,125,63,191,14,124,63,0,0,160,65,0,0,32,66,0,0,160,65,0,0,32,66,0,0,32,66,0,0,160,65,0,0,32,66,0,0,160,65,8,0,2,0,236,81,133,66,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,100,254,131,137,87,88,0,9,0,7,240,150,61,0,0,0,0,0,0,0,0,28,124,225,61,52,128,215,63,0,0,160,64,0,0,0,0,0,0,0,0,205,204,12,62,103,213,39,62,230,63,76,192,0,0,0,0,0,0,0,0,145,237,60,191,251,58,64,63,177,80,131,64,0,0,0,0,0,0,0,0,93,254,227,62,54,60,133,191,0,0,64,64,12,0,10,0,0,0,0,0,0,0,0,0,13,5,11,0,0,0,2,97,212,217,189,123,211,184,190,246,39,132,190,206,174,109,189,251,75,175,189,235,9,110,62,137,144,36,63,45,8,80,62,144,77,210,188,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,255,226,62,40,255,226,190,0,0,0,0,0,0,0,0,76,31,165,190,5,133,25,190,99,111,16,191,4,102,151,189,223,240,98,190,35,221,96,62,233,47,232,61,154,195,212,62,246,23,39,191,0,0,0,0,208,204,147,189,31,212,43,190,235,102,187,62,96,223,37,190,68,35,41,190,176,189,140,62,167,195,139,189,61,247,59,62,197,184,64,62,0,0,0,0,244,158,240,189,150,236,38,62,220,212,82,190,97,85,116,190,38,131,133,189,226,168,44,62,210,144,202,190,155,4,251,62,111,28,141,62,0,0,0,0,11,238,37,61,214,142,233,189,152,81,180,190,225,50,209,62,51,229,221,62,153,207,193,59,0,126,171,60,100,47,212,62,12,59,73,189,0,0,0,0,109,51,81,189,246,41,221,189,14,235,164,190,106,152,64,62,146,87,64,62,211,57,245,189,85,105,18,61,201,169,91,190,254,132,14,189,0,0,0,0,67,219,100,62,66,204,199,190,41,243,253,189,179,13,234,189,8,59,224,190,29,6,33,190,164,176,176,190,54,130,42,63,55,59,158,189,0,0,0,0,180,113,104,190,83,10,224,190,121,202,43,190,103,45,12,190,15,201,28,190,45,147,66,63,59,77,166,189,87,205,216,189,202,231,80,190,0,0,0,0,61,78,135,190,204,10,107,190,83,139,36,62,193,61,191,62,98,160,17,190,189,93,7,63,134,130,186,61,225,40,223,189,104,13,99,190,0,0,0,0,255,48,206,190,218,86,40,189,67,21,240,190,140,32,28,61,216,22,56,190,200,133,35,190,235,148,37,62,54,40,19,63,59,144,196,190,0,0,0,0,56,41,1,191,129,1,168,190,155,197,38,190,19,130,161,190,172,193,237,189,76,39,22,62,156,12,115,63,153,230,241,59,251,43,182,190,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,159,49,7,191,186,149,18,63,0,0,0,0,0,0,0,0,24,104,156,190,241,167,235,189,0,0,0,0,0,0,0,0,42,27,1,190,234,50,155,62,0,0,0,0,0,0,0,0,104,247,151,189,48,189,192,62,0,0,0,0,0,0,0,0,223,179,175,190,87,168,137,190,0,0,0,0,0,0,0,0,44,240,99,62,155,142,95,191,0,0,0,0,0,0,0,0,74,14,53,63,152,160,21,191,0,0,0,0,0,0,0,0,115,135,204,62,33,73,249,190,0,0,0,0,0,0,0,0,138,35,98,191,36,48,73,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,2,88,154,50,72,197,13,232,75,9,255,180,75,145,98,131,75,167,32,87,73,174,158,62,73,8,35,45,73,12,163,26,72,242,227,55,72,126,235,71,72,0,0,0,0,0,0,0,0,0,0,0,0,184,21,18,72,175,32,249,75,207,100,195,75,164,64,141,75,117,176,79,73,115,35,54,73,215,19,36,73,36,167,240,71,157,166,10,72,81,152,20,72,0,0,128,63,0,0,128,63,0,0,128,63,0,0,0,88,1,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,8,7,8,7,8,7,8,7,8,7,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,255,255,255,255,255,255,255,255,255,255,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,112,23,112,23,112,23,112,23,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,5,10,5,0,2,0,10,0,30,0,5,0,5,0,5,0,5,0,5,0,5,0,64,1,100,0,100,0,100,0,200,0,200,0,200,0,64,1,64,1,64,1,10,1,0,0,0,0,183,167,0,0};
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
            fr = f_read(&fil, serialized_state, BSEC_MAX_WORKBUFFER_SIZE*sizeof(uint8_t), &bread);
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
        bsec_set_state(serialized_state, n_serialized_state, work_buffer_state, n_work_buffer_size);
        check_rslt_bsec( rslt_bsec, "BSEC_SET_STATE");
    }


    sleep_ms(1000);
    gpio_deinit(PIN_FORMAT_INPUT);
    gpio_deinit(PIN_FORMAT_OUTPUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

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
    lorawan_join_C();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    conf_bsec.next_call = BME68X_SLEEP_MODE;
    uint8_t number_samples = 0;
    // loop forever
    uint64_t last_send_time = 0; 
    while (1) {
        uint64_t currTimeNs = time_us_64()*1000;
        current_op_mode = conf_bsec.op_mode;
        //set to forced mode
        if(currTimeNs >= conf_bsec.next_call){
            rslt_bsec = bsec_sensor_control(currTimeNs, &conf_bsec);
            check_rslt_bsec(rslt_bsec, "BSEC_SENSOR_CONTROL");
            if(rslt_bsec != BSEC_OK)
                continue;
            if(conf_bsec.op_mode != current_op_mode){
            
                switch(conf_bsec.op_mode){
                    case BME68X_FORCED_MODE:
                        /*forced mode is note used with the sample rate*/
                        printf("--------------Forced Mode--------------\n");
                        break;
                    case BME68X_PARALLEL_MODE:
                        if (current_op_mode != conf_bsec.op_mode){

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
                        }
                        break;
                    case BME68X_SLEEP_MODE:
                        if (current_op_mode != conf_bsec.op_mode){
                            printf("--------------Sleep Mode--------------\n");
                            rslt_api = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme); 
                            current_op_mode = BME68X_SLEEP_MODE;
                            if(number_samples > 0 && (time_us_64() - last_send_time) > 3000000){
                                pkt.p1 /= number_samples;
                                pkt.p2 /= number_samples;
                                pkt.p3 /= number_samples;
                                pkt.p4 /= number_samples;
                            #ifdef DEBUG
                                printf("\n");
                                if (lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2) < 0) {
                                    printf("failed!!!\n");
                                } else {
                                    printf("success!\n");
                                }
                            #else
                                lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2) >= 0
                            #endif
                                
                            }
                            pkt.p1 = 0;
                            pkt.p2 = 0;
                            pkt.p3 = 0;
                            pkt.p4 = 0;
                        }
                        break;
                }
            }
            if(conf_bsec.trigger_measurement){
                //CLASS C sensor should never be in FORCED MODE
                
                if(conf_bsec.op_mode == BME68X_PARALLEL_MODE){
                    del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);
                    bme.delay_us(del_period, bme.intf_ptr);
                    
                    rslt_api = bme68x_get_op_mode(&current_op_mode, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_op_mode");

                    rslt_api = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, &bme);
                    check_rslt_api(rslt_api, "bme68x_get_data");
                    for(uint8_t data_idx = 0; data_idx<n_fields; data_idx++){
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
                                add_probabilites(&pkt, output[i].sensor_id, output[i].signal);
                                }
                            }
                        }
                    }
                }
            }
        }
        
        if (lorawan_process() == 0) { 
            // check if a downlink message was received
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
        }
    }
    save_state_file();
    return 0;
}

void add_probabilites(struct uplink* pkt, int id, float signal){
    switch(id){
        case BSEC_OUTPUT_GAS_ESTIMATE_1:
            pkt->p1 += signal;
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_2:
            pkt->p2 += signal;
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_3:
            pkt->p3 += signal;
            break;
        case BSEC_OUTPUT_GAS_ESTIMATE_4:
            pkt->p4 += signal;
            break;
        default:
            break;
    }
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
    rslt_bsec = bsec_get_state(0, serialized_state, n_serialized_state_max, work_buffer_state, n_work_buffer_size, &n_serialized_state);
    check_rslt_bsec(rslt_bsec, "BSEC_GET_STATE");
    fr = f_write(&fil, serialized_state, BSEC_MAX_PROPERTY_BLOB_SIZE*sizeof(uint8_t), &bwritten);
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
