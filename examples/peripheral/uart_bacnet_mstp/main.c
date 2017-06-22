/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"

#include "hardware.h"
#include "timer.h"
#include "rs485.h"
#include "led.h"

#include "user_bacnet.h"

/* local version override */
char *BACnet_Version = "1.0";

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    //uint32_t err_code;
	struct itimer Blink_Timer;
	
    timer_init();
    led_init();
    rs485_init();
    bacnet_init();
    timer_interval_start(&Blink_Timer, 125);	

    // This part of the example is just for testing the loopback .
    while (true)
    {
        if (timer_interval_expired(&Blink_Timer)) {
            timer_interval_reset(&Blink_Timer);
            led_ld3_toggle();
        }
        led_task();
        bacnet_task();					
    }		
}


/** @} */
