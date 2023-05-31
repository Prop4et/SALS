#include "bme68x_API.h"
#include <stdio.h>


static uint8_t dev_addr;
static i2c_inst_t *i2c = i2c0;


void delay_us(uint32_t period, void *intf_ptr){
    if (period >= 1000)
        sleep_ms(period/1000);
    else
        sleep_ms(1);
}

/**
 * @brief blinks the built in led
 */
void blink(){
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }
}

void check_rslt_api(int8_t rslt, const char api_name[], void (*func_ptr)(char*, uint32_t))
{
    if(rslt != BME68X_OK){
        char log[256];
        if(rslt < BSEC_OK){
            printf("BME68X error code: %d on function %s\n", rslt, api_name);
            sprintf(log, "BME68X error code: %d on function %s\n", rslt, api_name);
            if(func_ptr != NULL) 
                func_ptr(log, strlen(log)+1);
            blink();
        }else{
            printf("BME68X warning code: %d on function %s\n", rslt, api_name);
            sprintf(log, "BME68X warning code: %d on function %s\n", rslt, api_name);
        }
    }
}

void check_rslt_bsec(bsec_library_return_t rslt, const char api_name[], void (*func_ptr)(char*, uint32_t)){
    if(rslt != BSEC_OK){
        char log[256];
        if(rslt < BSEC_OK){
            printf("BSEC error code: %d on function %s\n", rslt, api_name);
            sprintf(log, "BME68X error code: %d on function %s\n", rslt, api_name);
            if(func_ptr != NULL) 
                func_ptr(log, strlen(log)+1);
            blink();
        }else{
            printf("BSEC warning code: %d on function %s\n", rslt, api_name);
            sprintf(log, "BME68X warning code: %d on function %s\n", rslt, api_name);
        }
        
    }
}

BME68X_INTF_RET_TYPE bme_write(uint8_t reg, const uint8_t *buf, uint32_t nbytes, void *intf_ptr){
    int num_bytes_read = 0;
    uint8_t msg[nbytes +1];
    int8_t ret = 0;
    if(nbytes<1){
        return -1;
    }

    msg[0] = reg;
    for(int i = 0; i < nbytes; i++){
        msg[i+1] = buf[i];
    }

    ret = i2c_write_blocking(i2c, dev_addr, msg, (nbytes+1), false);
    if(ret > 0)
        return 0;
    else
        return -1;
}


BME68X_INTF_RET_TYPE bme_read(uint8_t reg, uint8_t *buf, uint32_t nbytes, void *intf_ptr){
    int8_t num_bytes_read = 0;
    int8_t ret = 0;

    if(nbytes < 0){
        return num_bytes_read;
    }
    i2c_write_blocking(i2c, dev_addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, dev_addr, buf, nbytes, false);

    if(num_bytes_read > 0)
        return 0;
    else 
        return -1;
}

int8_t bme_interface_init(struct bme68x_dev *bme, uint8_t intf){
    //i2c interface configuration
    if (intf == BME68X_I2C_INTF){
        //bme with pin to gnd, i2c uses low address
        dev_addr = BME68X_I2C_ADDR_LOW;
        bme->read = bme_read;
        bme->write = bme_write;
        bme->intf = BME68X_I2C_INTF;
        //i2c baudrate ad 400KHz
        i2c_init(i2c, 400*1000);
        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    }
    /* Bus configuration : SPI */
    else if (intf == BME68X_SPI_INTF){
        //TODO: spi interface not used yet so not implemented
    }
    bme->intf_ptr = &dev_addr;
    bme->amb_temp = 20;
    bme->delay_us = delay_us;
    return 0;
}

uint8_t get_dev_addr(){
    return dev_addr;
}

i2c_inst_t* get_i2c(){
    return i2c;
}

