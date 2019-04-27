/*******************************************************************************
 *
 * Copyright (C) 2019 by Shilpi Gupta
 *
 ******************************************************************************/

/*
 * @file    adc.c
 * @brief   Library definitions for ADC on the FRDM KL25Z MCU.
 * @version Project 3
 * @date    April 29, 2019
 */

/*
NOTES:

Two types of I/O ports in the microcontroller:
- General Purpose I/O (GPIO): Used for interfacing with LEDs, switches, LCD,
  keypad, etc
- Special Purpose I/O: Have a designated function such as ADC, UART, Timers...

Two registers associated with each I/O port:
- Data register
- Direction register
*/

#include "adc.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

// Define static vars.
uint32_t my_buffer[NUM_SAMPLES];
int32_t dma_done = 0;

void adc_init()
{
    // Enable clock for PORT E (bit 13 = 0x2000).
    SIM->SCGC5 |= SIM_SCGC5_PORTE(1);

    // Enable clock for ADC0 module.
    // Clock register for ADC0 found in the SIM_SCGC6 (System Clock Gating
    // Control) register. ADC0 clock is at bit 27 = 0x08000000.
    SIM->SCGC6 |= SIM_SCGC6_ADC0(1);

    // Select func for PTE20 pin that corresponds to ADC. (Default = ADC0)
    // (PCR bits 10-8 = MUX, 000 => Pin disable ((analog)). 
    // (i.e. Make PTE20 the ADC0 pin) (000 = 0x00).
    PORTE->PCR[20] |= PORT_PCR_MUX(0);

    // Set single-ended mode (default) (other option is differential mode).
    // (bit 5 = 0x020 = DIFF) (single-ended = 0)
    ADC0->SC1[0] |= ADC_SC1_DIFF(0);

    // Set ADC bit resolution to 16-bit via the ADCx_CFG1 (ADC Configuration 1)
    // register. (bits 3-2 = 0xC = MODE, 11 => 16-bit conversion when DIFF = 0)
    // (0x11 = 3 dec) (mode can be 8, 10, 12, or 16)
    ADC0->CFG1 |= ADC_CFG1_MODE(3);

    // Set speed of clock source to the ADC via the ADCx_CFG1 (ADC Configuration
    // 1) register. (bits 1-0 = 0x3 = ADICLK (Input Clock Select), 00 => Bus
    // clock)
    ADC0->CFG1 |= ADC_CFG1_ADICLK(0);

    // Set clock divide select via the ADC via the ADCx_CFG1 (ADC Configuration
    // 1) register. (bits 6-5 = 0x060 = ADIV (Clock Divide Select: clock
    // divided by 2^ADIV) (sys clock ~= 20MHz, 20MHz / 2^3 = 2.5 MHz)
    ADC0->CFG1 |= ADC_CFG1_ADIV(3);

    // TODO: Is this needed??
    // 0: Hardware average function disabled.
    // 1: Hardware average function enabled.
    ADC0->SC3 |= ADC_SC3_AVGE_MASK;

    // TODO: Is this needed??
    // ADCO = Continuous Conversion Enable
    // 0: One conversion or one set of conversions if the hardware averag
    //    function is enabled, that is, AVGE=1, after initiating a conversion.
    // 1: Continuous conversions or sets of conversions if the hardware average
    //    function is enabled, that is, AVGE=1, after initiating a conversion.
    ADC0->SC3 |= ADC_SC3_ADCO_MASK;
}

void adc_init_dma()
{
    // Initialize clock for DMA multiplexer.
    SIM->SCGC6 |= SIM_SCGC6_DMAMUX(1);

    // Initialize clock for DMA.
    SIM->SCGC7 |= SIM_SCGC7_DMA(1);

    // TODO: Is this needed??
    // Enables conversion complete interrupts. When COCO becomes set while the
    // respective AIEN is high, an interrupt is asserted.
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

    // Set the conversion to occur on channel 0.
    // (bits 4-0 = ADCH = ADC input channel) (channel 0 = 00000)
    ADC0->SC1[0] |= ADC_SC1_ADCH(0);

    // DMA is enabled and will assert the ADC DMA request during an ADC
    // conversion complete event noted when any of the SC1n[COCO] flags is
    // asserted. (DMAEN = DMA Enable)
    ADC0->SC2 |= ADC_SC2_DMAEN(1);
  
    // Set DMA channel 0 to transfer samples from ADC to buffer.
    // (ENBL = 0x80 of DMAMUX_CHCFG (Channel Configuration) register)
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL(1);

    // Set DMA multiplexer to route DMA request from ADC to DMA channel 0.
    // (i.e. Set the source for this channel.)
    // Source module = ADC, Source number = 40, (bits 5-0 = SOURCE)
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40);

    // Set source address for channel to be ADC data register.
    //(i.e location of the peripheral data register)
    DMA0->DMA[0].SAR |= (uint32_t)&ADC0->R[0];

    // Set destination address as beginning of buffer.
    DMA0->DMA[0].DAR |= (uint32_t)&my_buffer;

    // Set CS = Cycle Steal.
    // 0: DMA continuously makes read/write transfers until the BCR decrements
    //    to 0. 
    // 1: Forces a single read/write transfer per request.
    DMA0->DMA[0].DCR |= DMA_DCR_CS(0);

    // Set source data size to be 1, 2, or 4 bytes.
    // (i.e. Size of source bus cycle of the DMA controller.)
    // (bits 21-20 = Source size, 00 => 32 bit, 10 => 16 bit)
    DMA0->DMA[0].DCR |= DMA_DCR_SSIZE(0);

    // Set destination data size to be 1, 2, or 4 bytes.
    // (i.e. Size of destination bus cycle for the DMA controller.)
    // (bits 18-17 = Destination size, 00 => 32 bit, 10 => 16 bit)
    DMA0->DMA[0].DCR |= DMA_DCR_DSIZE(0);

    // Number of bytes to be transferred in each service request of the channel.
    // The DMA controller stops after a specified number of transfers.    
    DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_BCR(BYTE_COUNT);

    // Set source to have no address update after each transfer.
    // (SINC = Source Increment, 0 => No change to SAR after a successful
    // transfer)
    DMA0->DMA[0].DCR |= DMA_DCR_SINC(0);

    // Set destination with address increment.
    // Controls whether destination address increments after each transfer.
    // (DINC = Destination Increment, 1 => The DAR increments by 1,2,4
    // depending on size of transfer).
    DMA0->DMA[0].DCR |= DMA_DCR_DINC(1);
    
    // Enable interrupt (EINT) on completion of transfer.
    // Determines whether an interrupt is generated by completing a transfer or
    // by the occurrence of an error condition.
    // 0: No interrupt is generated.
    // 1: Interrupt signal is enabled.
    DMA0->DMA[0].DCR |= DMA_DCR_EINT(1);
    
    // TODO: ??
    // Enable asynchronous DMA requests
    // Enables the channel to support asynchronous DREQs while the MCU is in
    // Stop mode.
    //DMA0->DMA[0].DCR = (DMA0->DMA[0].DCR & (~DMA_DCR_EADREQ_MASK)) | DMA_DCR_EADREQ(1);
    //DMA0->DMA[0].DCR = DMA_DCR_EADREQ(1);

    // Set the DMA_ERQ (Enable Request) to be able to initiate a DMA transfer
    // when a hardware service request is issued.
    // CAUTION: Be careful: a collision can occur between the START bit and
    // D_REQ when the ERQ bit is 1.
    // 1: Enables peripheral request to initiate transfer. A software-initiated
    // request (setting the START bit) is always enabled.
    DMA0->DMA[0].DCR |= DMA_DCR_ERQ_MASK;

    // For debugging...
    PRINTF("In init(), addr of ADC->R[0] is: %p\r\n", &ADC0->R[0]); // addr of adc data reg
    PRINTF("In init(), addr stored in DMA0->DMA[0].SAR is p: %p\r\n\r\n", DMA0->DMA[0].SAR); // addr stored in reg
    PRINTF("In init(), addr of buffer is: %p\r\n", &my_buffer); // addr of buffer reg
    PRINTF("In init(), addr stored in DMA[0]->DAR is: %p\r\n\r\n", DMA0->DMA[0].DAR); // addr stored in reg

    // Start the transfer.
    // 0: DMA inactive
    // 1: The DMA begins the transfer in accordance to the values in the TCDn.
    //    START is cleared automatically after one module clock and always
    //    reads as logic 0.
    DMA0->DMA[0].DCR |= DMA_DCR_START(1);

    // Enable the interrupt for DMA0.
    NVIC_EnableIRQ(DMA0_IRQn);
}

int adc_did_complete_convert()
{
    // Check the COCO (Conversion Complete) flag in the ADC0_SC1A (Status
    // Control) register. (bit 7 = 0x080) When this flag goes high, the
    // conversion from analog to digital is complete and the value in the
    // ADC0_R0 register can be read. (The COCO flag is cleared automatically
    // when the data from the ADCx_Rn register is read.)
    if (ADC0->SC1[0] & ADC_SC1_COCO(1))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint16_t adc_get_digital_output()
{
   // The lower 16 bits of ADC0_RA contain the unsigned int result.
   return (uint16_t)ADC0->R[0];
}

uint16_t adc_get_digital_output_blocking()
{
    while (!adc_did_complete_convert()) {}

    return adc_get_digital_output();
}

// Attribution: https://www.reddit.com/r/C_Programming/comments/5hczva/integer_in_c_and_how_to_print_its_binary/
void printBits(char *name, uint32_t num)
{
    unsigned int mask = 1 << ((sizeof(int) << 3) - 1);
    PRINTF("%s", name);
    while (mask) {
        PRINTF("%d", (num&mask ? 1 : 0));
        mask >>= 1;
    }
    PRINTF("\r\n");
}

void DMA0_IRQHandler(void)
{
    /*
    DSR_BCR DONE flag:
    Set when all DMA controller transactions complete as determined by transfer
    count, or based on error conditions. When BCR reaches zero, DONE is set when
    the final transfer completes successfully. DONE can also be used to abort a
    transfer by resetting the status bits. When a transfer completes, software
    must clear DONE before reprogramming the DMA.
    0: DMA transfer is not yet complete. Writing a 0 has no effect.
    1: DMA transfer completed. Writing a 1 to this bit clears all DMA status
       bits and should be used in an interrupt service routine to clear the DMA
       interrupt and error bits
    */

    // For debugging...
    PRINTF("In IRQ(), addr of buffer is: %p\r\n", &my_buffer); // addr of buffer reg
    PRINTF("In IRQ(), addr stored in DMA[0]->DAR is: %p\r\n", DMA0->DMA[0].DAR); // addr stored in reg

    if (DMA0->DMA[0].DSR_BCR & DMA_DSR_BCR_DONE(1))
    {
  	// Clear the DMA interrupt and error bits.
	DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_DONE(1);

        // Reset destination address to beginning.
	DMA0->DMA[0].DAR |= (uint32_t)&my_buffer[0];

	// Reset BCR size.
	DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_BCR(BYTE_COUNT);

	dma_done = 1;
    }
}
