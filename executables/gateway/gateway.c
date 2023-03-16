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

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "lora-config.h"

// pin configuration for SX1276 radio module
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

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

int main( void )
{
    char devEui[17];

    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    sleep_ms(10000);

    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }
    
    printf("Pico LoRaWAN - Default Dev EUI = %s\n", lorawan_default_dev_eui(devEui));


    // uncomment next line to enable debug
    lorawan_debug(true);

    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    
    printf("Erasing NVM ... ");
    if (lorawan_erase_nvm() < 0) {
        printf("failed!!!\n");
    } else {
        printf("success!\n");
    }

    if(lorawan_init(&sx12xx_settings, LORAWAN_REGION) < 0){
        printf("Error during lorawan init\n");
    }

    uint32_t last_message_time = 0;

    // loop forever
    while (1) {
        // let the lorwan library process pending events
        lorawan_process();

        // get the current time and see if 5 seconds have passed
        // since the last message was sent
       

        // check if a downlink message was received
        receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
        if (receive_length > -1) {
            printf("received a %d byte message on port %d: ", receive_length, receive_port);

            for (int i = 0; i < receive_length; i++) {
                printf("%02x", receive_buffer[i]);
            }
            printf("\n");
        }
    }

    return 0;
}
