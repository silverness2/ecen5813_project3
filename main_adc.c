/*******************************************************************************
 *
 * Copyright (C) 2019 by Shilpi Gupta
 *
 ******************************************************************************/

/*
 * @file    main_adc..c
 * @brief   A program using ADC, DMA, DSP on the Freedom Freescale (FRDM)
 *          KL25Z Microcontroller (MCU).
 * @version Project 3
 * @date    May 1, 2019
 *
 * NOTES:
 * - Project run in the MCUXpresso IDE using SDK_2.x_FRDM_KL25Z version 2.0.0.
 * - On Ubuntu, look for serial ports by listing ports by: ls -l /sys/class/tty*
 *   (Can unplug and plug device to confirm which is the active serial port for
 *   the device).
 * - Can use putty or screen for terminal emulation.
 * - For example: sudo screen /dev/ttyACM1 115200
 *
 * 2^16 = 65536. So, scale on a 16-bit ADC is [0, 65535] counts (i.e. raw
 * sample ADC values). Ex. If input voltage=1V and ref voltage=3V, the
 * result, no matter if 8, 16, or 24 bit resolution, is 1/3.  The better the
 * ADC resolution, the better you can distinguish bw 1.001V and 1.002V.
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "adc.h"

//#define TEST_LED
//#define USE_BLOCKING
#define USE_INTERRUPT
#define SHOW_BUFFER_VALUES
#define NUM_SHOW_VALS 5
#define SHOW_DBFS
#define DELAY_MS 100


void delay(int ms)
{
    int i, j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 7000; j++)
        {}
    }
}

int main(void) {

    // Init board hardware.
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    // Init FSL debug console.
    BOARD_InitDebugConsole();

    PRINTF("\r\nWelcome to the ADC Program!\r\n\r\n");

#ifdef TEST_LED
    // Initialize blue led.
    led_blue_init();

    // Toggle blue led.
    while (1)
    {
        set_led_blue_on();
        delay(DELAY_MS);
        set_led_blue_off();
        delay(DELAY_MS);
    }
#endif

#ifdef USE_BLOCKING
    // Initialize ADC hardware.
    adc_init();

    static uint16_t d_out;
    while (1)
    {
    	// Set the conversion to occur on channel 0.
    	// TODO: Using |= ADC_SC1_ADCH(0) causes it to NOT work - WHY?
        ADC0->SC1[0] = ADC_SC1_ADCH(0);

        // Get the digital output after an analog to digital conversion occurs.
        d_out = adc_get_digital_output_blocking();

        PRINTF("ADC value: %u\r\n", d_out);
    }
#endif

#ifdef USE_INTERRUPT

    // Initialize DMA and ADC.
    adc_init();
    adc_dma_init();
    dma_init();

    // Initialize blue led.
    led_blue_init();

    while (1)
    {
        if (dma_done)
        {
#ifdef SHOW_BUFFER_VALUES
	    // Print buffer values.
       	    int i;
       	    if (buff_num == 1)
       	    {
                PRINTF("\r\nBuffer 0:\r\n");
	        for (i = 0; i < NUM_SHOW_VALS; i++)
	        {
	            PRINTF("buffer[%i]: %u\r\n", i, my_buffer[i]);
	        }
            }
            else
            {
                PRINTF("\r\nBuffer 1:\r\n");
	        for (i = (NUM_SAMPLES/2); i < (NUM_SAMPLES/2) + NUM_SHOW_VALS; i++)
	        {
	            PRINTF("buffer[%i]: %u\r\n", i, my_buffer[i]);
	        }
       	    }
            PRINTF("...\r\n");
#endif
            // Compute peak level.
            compute_peak_level();
            PRINTF("Peak level for buffer %i is: %u\r\n", !buff_num, curr_peak);

#ifdef SHOW_DBFS
            // Compute dbfs and show.
            show_dbfs();
#endif
            dma_done = 0;

	    // Enable the interrupt for DMA0.
	    NVIC_EnableIRQ(DMA0_IRQn);
        }
    }
#endif

    return 0;
}
