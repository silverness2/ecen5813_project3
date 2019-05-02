/*******************************************************************************
 *
 * Copyright (C) 2019 by Shilpi Gupta
 *
 ******************************************************************************/

/*
 * @file    adc.h
 * @brief   Library declarations for ADC on the FRDM KL25Z MCU.
 * @version Project 3
 * @date    May 1, 2019
 */

#include <stdint.h>

#ifndef __ADC_H
#define __ADC_H

#define NUM_SAMPLES 256
#define BYTE_COUNT ((NUM_SAMPLES/2) * sizeof(uint32_t)) // gives 512 bytes
#define DECAY_COEFF 0.90

extern uint32_t my_buffer[NUM_SAMPLES];
extern int32_t dma_done;
extern int32_t buff_num;

extern uint32_t curr_peak;

// Functions.
void adc_init();
void adc_dma_init();
void dma_init();
int adc_did_complete_convert();
uint16_t adc_get_digital_output();
uint16_t adc_get_digital_output_blocking();
void compute_peak_level();
void show_dbfs();
void DMA0_IRQHandler(void);
float log_2(uint32_t value);
void printBits(char *name, uint32_t num);

#endif
