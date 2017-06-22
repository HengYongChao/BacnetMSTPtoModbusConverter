/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"

#include "hardware.h"
#include "timer.h"
#include "rs485.h"
#include "led.h"
#include "user_bacnet.h"

/* local version override */
char *BACnet_Version = "1.0";


/**
 * @brief Function for application main entry.
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
