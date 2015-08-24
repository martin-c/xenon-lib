/*! \file
 *  adc.h
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
 */

#pragma once

#include <inttypes.h>
#include <avr/io.h>
#include "semaphore.h"



/***        Public Variables            ***/

/* Double-buffered accumulator for each ADC channel used.
 */
struct adcChanBuffer_s {
    volatile int16_t _acc[2];
    volatile int16_t *_act;
    volatile uint8_t _rc;
    uint8_t _sc;
};

/*! ADC voltage reference configuration enum */
enum adcVoltageRef_e {
    ADC_VREF_INT1V  = ADC_REFSEL_INT1V_gc,  ///< Internal 1.00V reference
    ADC_VREF_INTVCC = ADC_REFSEL_VCC_gc,    ///< Internal Vcc / 1.6
    ADC_VREF_AREFA  = ADC_REFSEL_AREFA_gc,  ///< External PORTA AREF
    ADC_VREF_AREFB  = ADC_REFSEL_AREFB_gc,  ///< External PORTB AREF
};

/*! ADC clock prescale configuration enum */
enum adcClkPrescale_e {
    ADC_CLK_DIV4    = ADC_PRESCALER_DIV4_gc,
    ADC_CLK_DIV8    = ADC_PRESCALER_DIV8_gc,
    ADC_CLK_DIV16   = ADC_PRESCALER_DIV16_gc,
    ADC_CLK_DIV32   = ADC_PRESCALER_DIV32_gc,
    ADC_CLK_DIV64   = ADC_PRESCALER_DIV64_gc,
    ADC_CLK_DIV128  = ADC_PRESCALER_DIV128_gc,
    ADC_CLK_DIV256  = ADC_PRESCALER_DIV256_gc,
    ADC_CLK_DIV512  = ADC_PRESCALER_DIV512_gc,
};

/*! ADC channel interrupt configuration enum */
enum adcChanIntLevel_e {
    ADC_CH_INT_OFF  = ADC_CH_INTLVL_OFF_gc,
    ADC_CH_INT_LO   = ADC_CH_INTLVL_LO_gc,
    ADC_CH_INT_MED  = ADC_CH_INTLVL_MED_gc,
    ADC_CH_INT_HI   = ADC_CH_INTLVL_HI_gc,
};

/*! ADC channel start conversion mask
 */
enum adcStartConversion_e {
    ADC_CH_0_START  = (1<<2),
    ADC_CH_1_START  = (1<<3),
    ADC_CH_2_START  = (1<<4),
    ADC_CH_3_START  = (1<<5),
};



/***        Public Functions            ***/

void adcInitSingleShot(struct ADC_struct *adc,
        enum adcVoltageRef_e vref,
        enum adcClkPrescale_e prescale);

void adcInitChannelDiffNoGain(struct ADC_CH_struct *ch,
        struct adcChanBuffer_s *buf,
        struct semaphore_s *sem,
        uint8_t samples,
        enum adcChanIntLevel_e int_lvl,
        uint8_t muxPos,
        uint8_t muxNeg);

void adcEnable(struct ADC_struct *adc);
void adcDisable(struct ADC_struct *adc);
void adcStartConversion(struct ADC_struct *adc, enum adcStartConversion_e ch_mask);
int16_t adcGetAccumulator(struct adcChanBuffer_s *buf);


