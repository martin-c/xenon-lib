/*! \file
 *  adc.c
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Martin Clemons
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * This module contains functions for configuring and operating the XMEGA ADC.
 * Each device has up to two ADCs, ADCA and ADCB, on the respective device ports.
 * We only utilize ADCA in this module, but adding access to ADCB would be simple,
 * see comments below.
 * Each ADC is divided into 4 channels, and it is possible to configure each channel
 * individually. On a programmatic level this module provides for each channel having
 * its own double-buffered accumulator with semaphore controlled access.
 * When a single ADC conversion is complete, an interrupt is generated and the conversion
 * result is added to the active accumulator buffer. This process repeats until the
 * desired sample count has been stored in the accumulator, and this condition automatically
 * swaps buffers, resets the desired sample count, and posts the associated semaphore.
 * The application then has until the next desired sample count is reached to retrieve
 * the data stored in the first accumulator. If this is not accomplished before this time then
 * the data in the accumulator will be overwritten, regardless of the semaphore state. In
 * short: The driver only *posts* the semaphore, it ignores the state of the semaphore
 * when writing data.
 */

#include "adc.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>



/***            Definitions             ***/
/*! \privatesection */

/***        Private Variables           ***/
/* Pointers to track accumulator assigned to each of the 4 ADC channels for ADCA.
 * Add adcb_acc array here if ADCB is used.
 */
static struct adcChanBuffer_s *adca_buf[4];

/* Pointers to track semaphores assigned to each of the 4 ADC channels for ADCA.
 * Add adcb_sem array here is ADCB is used.
 */
static struct semaphore_s *adca_sem[4];



/***        Public Variables            ***/
/*! \publicsection */



/***        Private Functions           ***/
/*! \privatesection */
static inline void update_buf(struct adcChanBuffer_s *buf, struct semaphore_s *sem, int8_t adc_result);
static void reset_buf(struct adcChanBuffer_s *buf);


/* Update ADC accumulator struct with new value.
 * If number of samples in accumulator has reached the desired sample count, swap buffers and post semaphore.
 * Function is called from within interrupt context.
 */
static inline void update_buf(struct adcChanBuffer_s *buf, struct semaphore_s *sem, int8_t adc_result)
{
    // add ADC result to active accumulator
    *buf->_act += adc_result;
    // update remaining sample count
    if (--buf->_rc == 0) {
        // desired sample count reached
        // swap accumulators
        buf->_act = (buf->_act == &buf->_acc[0]) ? &buf->_acc[1] : &buf->_acc[0];
        *buf->_act = 0;         // clear active accumulator
        buf->_rc = buf->_sc;    // reset remaining sample count
        up(sem);                // post semaphore
    }
}

/* Reset a buffer.
 * Sets active accumulator to first buffer, resets active accumulator to 0, and resets remaining sample count.
 */
static void reset_buf(struct adcChanBuffer_s *buf)
{
    buf->_act = buf->_acc;      // set first accumulator active
    *buf->_act = 0;             // clear active accumulator
    buf->_rc = buf->_sc;        // reset remaining sample count
}



/***                ISR                 ***/

/* Interrupts call the general update_buf() function with parameters specific to the ADC channel which
 * generated the interrupt. If ADCB were to be used interrupt handlers for this ADC would need to be added
 * here.
 * Function calls assume ADC is in 8-bit signed mode.
 */
ISR(ADCA_CH0_vect)
{
    update_buf(adca_buf[0], adca_sem[0], (int8_t) ADCA.CH0.RESL);
}

ISR(ADCA_CH1_vect)
{
    update_buf(adca_buf[1], adca_sem[1], (int8_t) ADCA.CH1.RESL);
}

ISR(ADCA_CH2_vect)
{
    update_buf(adca_buf[2], adca_sem[2], (int8_t) ADCA.CH2.RESL);
}

ISR(ADCA_CH3_vect)
{
    update_buf(adca_buf[3], adca_sem[3], (int8_t) ADCA.CH3.RESL);
}



/***        Public Functions            ***/
/*! \publicsection */

/*! Initialize ADC for "single shot" reading mode - where a single reading is taken when
 *  adcStartConversion is called.\n
 *  ADC is initialized in 8-bit mode. See XMEGA errata for reasons why 12-bit mode is not supported.
 *  \param adc Pointer to ADC hardware peripheral, ADCA or ADCB. NOTE: Only ADCA is currently supported.
 *  \param vref ADC reference voltage selection.
 *  \param prescale ADC clock prescaler.
 */
void adcInitSingleShot(struct ADC_struct *adc,
        enum adcVoltageRef_e vref,
        enum adcClkPrescale_e prescale)
{
    adc->CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_8BIT_gc;   // ADC signed mode, 8-bit resolution
    adc->REFCTRL = vref;                                    // set ADC voltage reference
    if (vref == ADC_VREF_INT1V) {
        adc->REFCTRL |= ADC_BANDGAP_bm;                     // enable bandgap if necessary
    }
    adc->PRESCALER = prescale;                              // set ADC prescaler
    // read ADC calibration values from NVM production signature row, store value in ADC calibration registers.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;                // load ADC calibration values
        // get address of ADCACAL0, ADCACAL1
        ADCA.CALL = pgm_read_byte(&((NVM_PROD_SIGNATURES_t *)0)->ADCACAL0);
        ADCA.CALH = pgm_read_byte(&((NVM_PROD_SIGNATURES_t *)0)->ADCACAL1);
        NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    }
}

/*! Initialize an ADC channel in differential mode with no gain.
 *  \param ch Pointer to ADC channel to initialize. NOTE: only channels on ADCA are currently supported!
 *  \param acc Pointer to buffer allocated for this channel.
 *  \param sem Pointer to semaphore allocated for this channel.
 *  \param samples Number of ADC conversions summed in accumulator before semaphore is posted. Set to 0 for 256 samples.
 *  \param int_lvl Channel conversion complete interrupt level. ISR is used to update channel accumulator.
 *  \param muxPos Positive input pin number. See table 25-12 in A3 datasheet. In this channel configuration
 *   pin number maps directly to bit value.
 *  \param muxNeg Negative input pin number. See table 25-14 in A3 datasheet. In this channel configuration
 *   pin number maps directly to bit value.
 */
void adcInitChannelDiffNoGain(struct ADC_CH_struct *ch,
        struct adcChanBuffer_s *buf,
        struct semaphore_s *sem,
        uint8_t samples,
        enum adcChanIntLevel_e int_lvl,
        uint8_t muxPos,
        uint8_t muxNeg)
{
    // compute channel number
    uint8_t c;
    if (ch == &ADCA.CH0) {
        c = 0;
    } else if (ch == &ADCA.CH1) {
        c = 1;
    } else if (ch == &ADCA.CH2) {
        c = 2;
    } else {
        c = 3;
    }
    adca_buf[c] = buf;                          // set pointer to accumulator for ISR
    adca_sem[c] = sem;                          // set pointer to semaphore for ISR
    buf->_sc = samples;                         // set desired sample count
    reset_buf(buf);                             // reset buffer
    ch->CTRL = ADC_CH_INPUTMODE_DIFF_gc;        // channel mode differential
    ch->INTCTRL = int_lvl;                      // set interrupt level
    // set channel mux
    ch->MUXCTRL = (muxPos & 0x0F) << 3 | (muxNeg & 0x03);
}

/*! Enable ADC. Sets ADC enable bit and flushes ADC.
 *  Resets channel buffers for all initialized channels.
 *  \param adc Pointer to ADC hardware peripheral, ADCA or ADCB. NOTE: Only ADCA is currently supported.
 */
void adcEnable(struct ADC_struct *adc)
{
    uint8_t i;

    // reset accumulators and status variables for all channels
    if (adc == &ADCA) {
        for (i = 0; i < sizeof adca_buf / sizeof adca_buf[0]; i++) {
            if (adca_buf[i] == 0) {
                continue;                       // don't reset un-initialized channels
            }
            reset_buf(adca_buf[i]);
        }
    } else {
        // same init but for ADCB goes here */
    }
    // flush and enable ADC
    adc->CTRLA |= ADC_FLUSH_bm | ADC_ENABLE_bm;
}

/*! Disable ADC. Clears ADC enable bit\n
 *  Note: any pending conversions are not cleared and will resume when ADC is enabled.
 *  \param adc Pointer to ADC hardware peripheral, ADCA or ADCB. NOTE: Only ADCA is currently supported.
 */
void adcDisable(struct ADC_struct *adc)
{
    adc->CTRLA &= ~(ADC_ENABLE_bm);
}

/*! Start ADC conversion for channel specified\n
 *  Specifying multiple channels will start a sweep. ADC must be previously initialized and enabled, and channel(s)
 *  must be initialized and enabled.
 *  \param adc Pointer to ADC hardware peripheral, ADCA or ADCB. NOTE: Only ADCA is currently supported.
 *  \param ch_mask ADC start conversion channel mask.
 */
void adcStartConversion(struct ADC_struct *adc, enum adcStartConversion_e ch_mask)
{
    adc->CTRLA |= ch_mask;              // set conversion start bits
}

/*! Get signed ADC accumulator value.
 *  Function should only be called after channel semaphore posts. Semaphore may be acquired before or after
 *  function is called since accumulator is double-buffered.
 *  \param buf Channel buffer.
 *  \return Signed channel accumulator value.
 */
int16_t adcGetAccumulator(struct adcChanBuffer_s *buf)
{
    volatile int16_t *acc;      // accumulator to return

    // select the non-active buffer
    acc = (buf->_act == &buf->_acc[0]) ? &buf->_acc[1] : &buf->_acc[0];
    return *acc;    // return accumulator value
}

