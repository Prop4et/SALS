#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include <stdio.h>
#include <string.h>
#include "lora-config.h"

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

int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

struct uplink {//modified from 16 bits to 32 cause of expected 18 bytes
    uint32_t id; 
    int32_t temp; //-273.15 - 90.00 -> -27315 - 9000
    uint32_t hum; //0.00 - 100.00 -> 0 - 10000
    uint32_t press; //840.0 - 1013.25 [hPa] -> 8400 - 10133
    uint32_t AQI; //50.0 - 500.0 -> 500 - 5000
    uint32_t CO2; //600 - 10000 [ppm] -> 600 - 10000
};

int main( void )
{
    struct uplink pkt = {
        .id = DEV_ID,
        .temp = 0,
        .hum = 0,
        .press = 0,
        .AQI = 0,
        .CO2 = 0,
    }; 
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    sleep_ms(5000);

    printf("Pico LoRaWAN - Hello ABP\n\n");

    // uncomment next line to enable debug
    lorawan_debug(true);
    lorawan_erase_nvm();
    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_abp(&sx12xx_settings, LORAWAN_REGION, &abp_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ... ");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    printf("joined successfully!\n");

    uint32_t last_message_time = 0;

    // loop forever
    while (1) {
        // let the lorwan library process pending events
        lorawan_process();

        // get the current time and see if 5 seconds have passed
        // since the last message was sent
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if ((now - last_message_time) > 30000) {
            // try to send an unconfirmed uplink message
            if (lorawan_send_unconfirmed(&pkt, sizeof(struct uplink), 2) < 0) {
                printf("failed!!!\n");
            } else {
                printf("success!\n");
            }

            last_message_time = now;
        }

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