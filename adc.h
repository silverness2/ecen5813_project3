/*******************************************************************************
 *
 * Copyright (C) 2019 by Shilpi Gupta
 *
 ******************************************************************************/

/*
 * @file    adc.h
 * @brief   Library declarations for ADC on the FRDM KL25Z MCU.
 * @version Project 3
 * @date    April 29, 2019
 */

#include <stdint.h>

#ifndef __ADC_H
#define __ADC_H

#define NUM_SAMPLES 256
#define BYTE_COUNT ((NUM_SAMPLES/2) * sizeof(uint32_t))

extern uint32_t my_buffer[NUM_SAMPLES];
extern int32_t dma_done;

// Functions.
void adc_init();
void adc_init_dma();
int adc_did_complete_convert();
uint16_t adc_get_digital_output();
uint16_t adc_get_digital_output_blocking();
void printBits(char *name, uint32_t num);

#endif
