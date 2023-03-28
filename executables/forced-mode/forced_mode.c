/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "../lib/bme/bme68x/bme68x.h"
#include "../lib/bme/bme_api/bme68x_API.h"

/***********************************************************************/
/*                         Macros                                      */
/***********************************************************************/

/* Macro for count of samples to be displayed */
#define SAMPLE_COUNT  UINT16_C(300)

/***********************************************************************/
/*                         Test code                                   */
/***********************************************************************/

int main(void)
{   
    stdio_init_all();
    sleep_ms(5000);
    struct bme68x_dev bme;
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 1;

    /* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF
     */
    rslt = bme_interface_init(&bme, BME68X_I2C_INTF);
    check_rslt_api(rslt, "bme68x_interface_init");
    uint8_t data_id;
    bme.read(BME68X_REG_CHIP_ID, &data_id, 1, bme.intf_ptr);
    if(data_id != BME68X_CHIP_ID){
        printf("Cannot communicate with BME688\n");
        printf("CHIP_ID: %x \t ID_READ: %x\n", BME68X_CHIP_ID, data_id);
    }
    else{
        bme.chip_id = data_id;
        printf("Connection valid, DEVICE_ID: %x\n", bme.chip_id);
    }
    rslt = bme68x_init(&bme);
    check_rslt_api(rslt, "bme68x_init");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    check_rslt_api(rslt, "bme68x_set_conf");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    check_rslt_api(rslt, "bme68x_set_heatr_conf");

    printf("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");

    while (sample_count <= SAMPLE_COUNT)
    {
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        check_rslt_api(rslt, "bme68x_set_op_mode");

        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        time_ms = time_us_64()/1000;

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        check_rslt_api(rslt, "bme68x_get_data");

        if (n_fields){
            printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x\n",
                sample_count,
                (long unsigned int)time_ms,
                data.temperature,
                data.pressure,
                data.humidity,
                data.gas_resistance,
                data.status);
            sample_count++;
        }
    }

    return rslt;
}