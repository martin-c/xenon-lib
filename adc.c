/*! \file
 *  adc.c
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Martin Clemons
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
 */

#include "adc.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>


/***            Definitions             ***/


/***        Private Variables           ***/

/* Buffered accumulator for each ADC channel, 256 signed 8 bit readings max.
 * Array size must match number of ADC channels in use.
 */
struct adcChanAcc_s {
    int16_t a[2];   // buffered accumulator
    uint8_t o;      // offset to determine whether primary or secondary accumulator is active
    uint8_t count;  // read count for channel
};

/* One accumulator for each channel used */
static volatile struct adcChanAcc_s adcChanAcc[2];



/***        Public Variables            ***/

/* ADC channel ready state */
volatile enum adcChAccState_e adcChAccReadyF;



/***        Private Functions           ***/
static inline void updateAcc(volatile struct adcChanAcc_s *acc, int8_t adcReading, uint8_t rdyFlag);
static void resetAcc(volatile struct adcChanAcc_s *acc);



/* Update ADC accumulator struct with new value
 *
 */
static inline void updateAcc(volatile struct adcChanAcc_s *acc, int8_t adcReading, uint8_t rdyFlag) {
    
    acc->a[acc->o] += adcReading;   // add 8 bit signed result to accumulator
    if (++acc->count == 0) {
        acc->o ^= 0x01;             // swap buffers if 256 readings have been collected
        acc->a[acc->o] = 0;         // clear new accumulator
        adcChAccReadyF |= rdyFlag;  // set channel ready flag
    }
}

static void resetAcc(volatile struct adcChanAcc_s *acc) {
    memset((char *)acc->a, 0, sizeof acc->a);
    acc->o = 0;
    acc->count = 0;
}



/***                ISR                 ***/
ISR(ADCA_CH0_vect) {
    updateAcc(&adcChanAcc[0], (int8_t)ADCA.CH0.RESL, ADC_CH0_READY);
}

ISR(ADCA_CH1_vect) {
    updateAcc(&adcChanAcc[1], (int8_t)ADCA.CH1.RESL, ADC_CH1_READY);
}
/*
ISR(ADCA_CH2_vect) {
    updateAcc(&adcChanAcc[2], (int8_t)ADCA.CH2.RESL, ADC_CH2_READY);
}

ISR(ADCA_CH3_vect) {
    updateAcc(&adcChanAcc[3], (int8_t)ADCA.CH3.RESL, ADC_CH3_READY);
}
*/


/***        Public Functions            ***/

/*! Initialize ADC for "single shot" reading mode - where a single reading is taken when
 *  adcStartConversion is called
 */
void adcInitSingleShot(enum adcVoltageRef_e vref,
                       enum adcClkPrescale_e ps) {
    
    ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_8BIT_gc;   // ADC signed mode, 8-bit resolution
    ADCA.REFCTRL = vref;                                    // set ADC voltage reference
    if (vref == ADC_VREF_INT1V) {
        ADCA.REFCTRL |= ADC_BANDGAP_bm;                     // enable bandgap if necessary
    }
    ADCA.PRESCALER = ps;                                    // set ADC prescaler
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;                    // load ADC claibration values
    ADCA.CALL = pgm_read_byte(0x20);
    ADCA.CALH = pgm_read_byte(0x21);
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
}

/*! Initialize an ADC channel
 *
 */
void adcInitChannelDiffNoGain(ADC_CH_t *ch,
                              enum adcChanInterrupt_e chInt,
                              uint8_t muxPos,
                              uint8_t muxNeg) {
    
    ch->CTRL = ADC_CH_INPUTMODE_DIFF_gc;        // channel mode differential
    ch->INTCTRL = chInt;                        // set interrupt level
    ch->MUXCTRL = muxPos << 3 | muxNeg;
}


/*! Enable ADC\n
 *  Sets ADC enable bit and flushes ADC
 */
void adcEnable(void) {
    
    uint8_t i;
    
    /* reset accumulators and status variables */
    for (i = 0; i < sizeof adcChanAcc / sizeof adcChanAcc[0]; i++) {
        resetAcc(&adcChanAcc[i]);
    }
    /* flush and enable ADC */
    ADCA.CTRLA |= ADC_FLUSH_bm | ADC_ENABLE_bm;
}



/*! Disable ADC\n
 *  Clears ADC enable bit\n
 *  Note: any pending conversions are not cleared and will resume when ADC is enabled
 */
void adcDisable(void) {
    
    ADCA.CTRLA &= ~(ADC_ENABLE_bm);
}



/*! Start ADC conversion for channel specified\n
 *  Specifiing multiple channels will start a sweep. ADC must be previously enabled.
 *  \param ch Bitmask of channel to start:\n
 *  bit 0 -> channel 0\n bit 1 -> channel 1\n bit 2 -> channel 2\n bit 3 -> channel 3\n
 */
void adcStartConversion(uint8_t ch) {
    
    if (ch > 3) {
        return;
    }
    ADCA.CTRLA |= (0x01 << (ch + 2));    // set conversion start bits
}



/*! Get 8 bit signed ADC result
 *  \return 8 bit signed CH0 result
 */
uint8_t adcGetResultCh0_8(void) {
    
    return ADCA.CH0.RESL;
}



/*! Get 8 bit signed ADC result
 *  \return 8 bit signed CH1 result
 */
uint8_t adcGetResultCh1_8(void) {
    
    return ADCA.CH1.RESL;
}



/*! Get 16 bit signed ADC accumulator for channel specified.\n
 *  Function returns the buffered ADC accumulator value.
 *  Should be called after ADC_CHx_READY flag is set by interrupt routine.
 *  Function clears ADC_CHx_READY flag.
 *  \param ch ADC Channel.
 *  \return 16 bit signed accumulator value
 */
int16_t adcGetAccumulator(uint8_t ch) {
    
    if (ch >= sizeof adcChanAcc / sizeof adcChanAcc[0]) {
        return 0;   // unsupported ch number
    }
    adcChAccReadyF &= ~(0x01 << ch);                // clear ADC_CHx_READY flag
    // return accumulator value of inactive buffer
    return adcChanAcc[ch].a[ adcChanAcc[ch].o ^ 0x01 ];
}



