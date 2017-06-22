/**************************************************************************
*
* Copyright (C) 2011 Steve Karg <skarg@users.sourceforge.net>
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Module Description:
* Handle the configuration and operation of the RS485 bus.
**************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "hardware.h"
#include "timer.h"
#include "bits.h"
#include "fifo.h"
#include "led.h"
#include "rs485.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "app_error.h"

/* buffer for storing received bytes - size must be power of two */
static uint8_t Receive_Buffer_Data[512];
static FIFO_BUFFER Receive_Buffer;
/* amount of silence on the wire */
static struct etimer Silence_Timer;
/* baud rate */
static uint32_t Baud_Rate = 38400;

/* The minimum time after the end of the stop bit of the final octet of a */
/* received frame before a node may enable its EIA-485 driver: 40 bit times. */
/* At 9600 baud, 40 bit times would be about 4.166 milliseconds */
/* At 19200 baud, 40 bit times would be about 2.083 milliseconds */
/* At 38400 baud, 40 bit times would be about 1.041 milliseconds */
/* At 57600 baud, 40 bit times would be about 0.694 milliseconds */
/* At 76800 baud, 40 bit times would be about 0.520 milliseconds */
/* At 115200 baud, 40 bit times would be about 0.347 milliseconds */
/* 40 bits is 4 octets including a start and stop bit with each octet */
#define 	Tturnaround  		(40UL)
#define		RS485_EN	 		(2UL)
#define 	RX_PIN_MODBUS		(1UL)
#define 	TX_PIN_MODBUS		(0UL)
#define 	RTS_PIN_NUMBER		(NULL)
#define 	CTS_PIN_NUMBER		(NULL)
#define 	UART_TX_BUF_SIZE 	5                         /**< UART TX buffer size. */
#define 	UART_RX_BUF_SIZE 	5  

bool volatile rs485_frame_sent_flag = false;
/*************************************************************************
* Description: Reset the silence on the wire timer.
* Returns: nothing
* Notes: none
**************************************************************************/
void rs485_silence_reset(
    void)
{
    timer_elapsed_start(&Silence_Timer);
}

/*************************************************************************
* Description: Determine the amount of silence on the wire from the timer.
* Returns: true if the amount of time has elapsed
* Notes: none
**************************************************************************/
bool rs485_silence_elapsed(
    uint32_t interval)
{
    return timer_elapsed_milliseconds(&Silence_Timer, interval);
}

/*************************************************************************
* Description: Baud rate determines turnaround time.
* Returns: amount of milliseconds
* Notes: none
**************************************************************************/
static uint16_t rs485_turnaround_time(
    void)
{
    /* delay after reception before transmitting - per MS/TP spec */
    /* wait a minimum  40 bit times since reception */
    /* at least 2 ms for errors: rounding, clock tick */
    if (Baud_Rate) {
        return (2 + ((Tturnaround * 1000UL) / Baud_Rate));
    } else {
        return 2;
    }
}

/*************************************************************************
* Description: Use the silence timer to determine turnaround time.
* Returns: true if turnaround time has expired.
* Notes: none
**************************************************************************/
bool rs485_turnaround_elapsed(
    void)
{
    return timer_elapsed_milliseconds(&Silence_Timer, rs485_turnaround_time());
}


/*************************************************************************
* Description: Determines if an error occured while receiving
* Returns: true an error occurred.
* Notes: none
**************************************************************************/
bool rs485_receive_error(
    void)
{
    return false;
}
#if 0
/*********************************************************************//**
 * @brief        USARTx interrupt handler sub-routine
 * @param[in]    None
 * @return         None
 **********************************************************************/
void USART2_IRQHandler(
    void)
{
    uint8_t data_byte;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        /* Read one byte from the receive data register */
        data_byte = USART_ReceiveData(USART2);
        (void) FIFO_Put(&Receive_Buffer, data_byte);
    }
}
#endif

/*************************************************************************
* DESCRIPTION: Return true if a byte is available
* RETURN:      true if a byte is available, with the byte in the parameter
* NOTES:       none
**************************************************************************/
bool rs485_byte_available(
    uint8_t * data_register)
{
    bool data_available = false;        /* return value */

    if (!FIFO_Empty(&Receive_Buffer)) {
        if (data_register) {
            *data_register = FIFO_Get(&Receive_Buffer);
        }
        timer_elapsed_start(&Silence_Timer);
        data_available = true;
        led_rx_on_interval(10);
    }

    return data_available;
}
/* ----------------------- Start implementation -----------------------------*/
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
	
	if(p_event->evt_type == APP_UART_TX_EMPTY)	
	{
		rs485_frame_sent_flag = true;
	}else
	{
		rs485_frame_sent_flag = false;
	}
//	if(p_event->evt_type == APP_UART_DATA_READY)
//	{
//        data_byte = USART_ReceiveData(USART2);
//        (void) FIFO_Put(&Receive_Buffer, data_byte);		
//	}
}
/*************************************************************************
* DESCRIPTION: Sends a byte of data
* RETURN:      nothing
* NOTES:       none
**************************************************************************/
void rs485_byte_send(
    uint8_t tx_byte)
{
    led_tx_on_interval(10);
    //USART_SendData(USART2, tx_byte);
	app_uart_put(tx_byte);	
    timer_elapsed_start(&Silence_Timer);
}

/*************************************************************************
* Description: Determines if a byte in the USART has been shifted from
*   register
* Returns: true if the USART register is empty
* Notes: none
**************************************************************************/
bool rs485_byte_sent(
    void)
{
    return nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY);	
	//return USART_GetFlagStatus(USART2, USART_FLAG_TXE);
}

/*************************************************************************
* Description: Determines if the entire frame is sent from USART FIFO
* Returns: true if the USART FIFO is empty
* Notes: none
**************************************************************************/
bool rs485_frame_sent(
    void)
{
    return rs485_frame_sent_flag;	
	//return USART_GetFlagStatus(USART2, USART_FLAG_TC);
}

/*************************************************************************
* DESCRIPTION: Send some data and wait until it is sent
* RETURN:      true if a collision or timeout occurred
* NOTES:       none
**************************************************************************/
void rs485_bytes_send(
    uint8_t * buffer,   /* data to send */
    uint16_t nbytes)
{       /* number of bytes of data */
    uint8_t tx_byte;

    while (nbytes) {
        /* Send the data byte */
        tx_byte = *buffer;
        /* Send one byte */
        //USART_SendData(USART2, tx_byte);
		app_uart_put(tx_byte);	
        while (!rs485_byte_sent()) {
            /* do nothing - wait until Tx buffer is empty */
        }
        buffer++;
        nbytes--;
    }
    /* was the frame sent? */
    while (!rs485_frame_sent()) {
        /* do nothing - wait until the entire frame in the
           Transmit Shift Register has been shifted out */
    }
    timer_elapsed_start(&Silence_Timer);

    return;
}

/*************************************************************************
* Description: Sets the baud rate to non-volatile storeage and configures USART
* Returns: true if a value baud rate was saved
* Notes: none
**************************************************************************/
bool rs485_baud_rate_set(
    uint32_t baud)
{
    bool valid = true;
    switch (baud) {
        case 9600:
			nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_9600);
		    break;
        case 19200:
			nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_19200);
			break;
        case 38400:
			nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_38400);
			break;
        case 57600:
			nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_57600);
			break;
        case 76800:
			nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_76800);
			break;
        case 115200:
			nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_115200);
            break;
        default:
            valid = false;
            break;
    }
    return valid;
}

/*************************************************************************
* Description: Determines the baud rate in bps
* Returns: baud rate in bps
* Notes: none
**************************************************************************/
uint32_t rs485_baud_rate(
    void)
{
    return Baud_Rate;
}

/*************************************************************************
* Description: Enable the Request To Send (RTS) aka Transmit Enable pin
* Returns: nothing
* Notes: none
**************************************************************************/
void rs485_rts_enable(
    bool enable)
{
    if (enable) {
        //GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
		nrf_gpio_pin_set(RS485_EN);
    } else {
        //GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
		nrf_gpio_pin_clear(RS485_EN);
    }
}

/*************************************************************************
* Description: Initialize the room network USART
* Returns: nothing
* Notes: none
**************************************************************************/
void rs485_init(
    void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          TX_PIN_MODBUS,
          RX_PIN_MODBUS,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);
	printf(" uart init ok.\n\r");
    APP_ERROR_CHECK(err_code);
		
    /* enable the USART to generate interrupts */
    //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//nrf_uart_int_enable(NRF_UART0, NRF_UART_INT_MASK_RXDRDY );
	
    rs485_baud_rate_set(Baud_Rate);

    //USART_Cmd(USART2, ENABLE);

    FIFO_Init(&Receive_Buffer, &Receive_Buffer_Data[0],
        (unsigned) sizeof(Receive_Buffer_Data));
    timer_elapsed_start(&Silence_Timer);
	
	nrf_gpio_cfg_output(RS485_EN);
}
