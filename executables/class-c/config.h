/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

// LoRaWAN region to use, full list of regions can be found at: 
//   http://stackforce.github.io/LoRaMac-doc/LoRaMac-doc-v4.5.1/group___l_o_r_a_m_a_c.html#ga3b9d54f0355b51e85df8b33fd1757eec
#define LORAWAN_REGION                  LORAMAC_REGION_EU868

#define DEV_EUI                        "e660c0d1c74a4530"

#define DEV_ID                          (uint16_t)4
// LoRaWAN device address (32-bit)
#define LORAWAN_DEV_ADDR                "xxxxxxxx"

// LoRaWAN Network Session Key (128-bit)
#define LORAWAN_NETWORK_SESSION_KEY     "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

// LoRaWAN Application Session Key (128-bit)
#define LORAWAN_APP_SESSION_KEY         "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

// LoRaWAN Channel Mask, NULL value will use the default channel mask 
// for the region
#define LORAWAN_CHANNEL_MASK            NULL

#ifdef DEBUG 
    #define INTERVAL          12  /*time between lora send in seconds*/
#else
    #define INTERVAL          3600 
#endif

/**
 * @brief utility to compute the time to wait in seconds semi exponentially
 * 
 * @param loops number of loops 
 * @param initial_wait initial waiting time
 * @param increment_factor 
 * @param cap max value reachable
 * @return int32_t the amount of time to wait in seconds
 */
/*static int32_t semi_exponential_wait_time(int loops, int initial_wait, int increment_factor, int cap){
    int32_t wait_time = (initial_wait * increment_factor)^loops;
    return wait_time < cap ? wait_time : cap;
}*/