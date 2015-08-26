/*! \file
 *  dma.c
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

#include "dma.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/atomic.h>



/***                Definitions                 ***/



/***         Private Global Variables           ***/

/* pointers to external DMA channel interrupt functions */
#ifdef XENON_DMA_LIB_ISR_HANDLER
void (*ch0Isr)(void) = 0;
void (*ch1Isr)(void) = 0;
void (*ch2Isr)(void) = 0;
void (*ch3Isr)(void) = 0;
#endif /* XENON_DMA_LIB_ISR_HANDLER */



/***          Public Global Variables           ***/



/***        Private function portotypes         ***/



/***                   ISRs                     ***/

#ifdef XENON_DMA_LIB_ISR_HANDLER
ISR(DMA_CH0_vect) {
    
    ch0Isr();
    DMA.CH0.CTRLB |= DMA_CH_TRNIF_bm;
}

ISR(DMA_CH1_vect) {
    
    ch1Isr();
    DMA.CH1.CTRLB |= DMA_CH_TRNIF_bm;
}

ISR(DMA_CH2_vect) {
    
    ch2Isr();
    DMA.CH2.CTRLB |= DMA_CH_TRNIF_bm;
}

ISR(DMA_CH3_vect) {
    
    ch3Isr();
    DMA.CH3.CTRLB |= DMA_CH_TRNIF_bm;
}
#endif /* XENON_DMA_LIB_ISR_HANDLER */


/***             Private Functions              ***/



/***             Public Functions               ***/

/*! Initialize DMA Controller
 *  \param dbm Double buffer mode selection.
 *  \param cp Channel priority selection.
 */
void dmaInit(enum dmaDoubleBufferMode_e dbm, enum dmaChannelPriority_e cp) {
    
    DMA.CTRL = dbm | cp;
}

/*! Enable DMA controller
 *
 */
void dmaEnable(void) {
    
    DMA.CTRL |= DMA_ENABLE_bm;
}

/*! Disable DMA controller
 *
 */
void dmaDisable(void) {
    
    DMA.CTRL &= ~(DMA_ENABLE_bm);
}

/*! Initialize DMA channel in single shot mode.
 *  \param c Pointer to DMA channel to configure.
 *  \param burstLen Length of channel burst transfer in bytes.
 *  \param srcReload Source address reload setting.
 *  \param srcDir Source address mode.
 *  \param destReload Destination address reload setting.
 *  \param destDir Destination address mode.
 *  \param tSource Channel trigger source.
 */
void dmaChInitSingleShot(struct DMA_CH_struct *c,
                         enum dmaChBurstLength_e burstLen,
                         enum dmaChAddressReload_e srcReload,
                         enum dmaChAddressMode_e srcDir,
                         enum dmaChAddressReload_e destReload,
                         enum dmaChAddressMode_e destDir,
                         enum DMA_CH_TRIGSRC_enum tSource) {
    
    // configure DMA channel registers
    c->CTRLA = DMA_CH_SINGLE_bm | burstLen;
    c->CTRLB = DMA_CH_ERRIF_bm | DMA_CH_TRNIF_bm;
    c->ADDRCTRL = srcReload << 6 | srcDir << 4 | destReload << 2 | destDir;
    c->TRIGSRC = tSource;
    c->TRFCNT = 0;
    c->REPCNT = 0;
}

/*! Register a DMA channel transaction complete ISR.
 *  \param c Pointer to DMA channel to configure.
 *  \param cmpIntLvl Transaction complete interrupt priority level.
 *  \param isr Pointer to channel interrupt function, 0 if none.
 */
#ifdef XENON_DMA_LIB_ISR_HANDLER
void dmaChRegisterIsr(struct DMA_CH_struct *c, 
                      enum dmaChTransactionCompleteInterrupt_e cmpIntLvl,
                      void (*isr)(void)) {
    
    // configure interrupt service routine
    c->CTRLB &= ~DMA_CH_TRNINTLVL_HI_gc;        // clear previous bits
    if (isr != NULL) {
        c->CTRLB |= cmpIntLvl;                  // set new bits
    }
    // assign function pointer to correct channel interrupt
    switch ((int16_t)c) {
        case (int16_t)&DMA.CH0:
            ch0Isr = isr;
            break;
        case (int16_t)&DMA.CH1:
            ch1Isr = isr;
            break;
        case (int16_t)&DMA.CH2:
            ch2Isr = isr;
            break;
        case (int16_t)&DMA.CH3:
            ch3Isr = isr;
            break;
        default:
            break;
    }
}
#endif /* XENON_DMA_LIB_ISR_HANDLER */

/*! Set the DMA channel repeat mode bit and repeat count
 *  \param c Pointer to DMA channel to configure.
 *  \param repCount Value to set DMA channel REPCNT register.
 */
void dmaChEnableRepeat(struct DMA_CH_struct *c, uint8_t repCount) {
    
    c->REPCNT = repCount;
    c->CTRLA |= DMA_CH_REPEAT_bm;
}

/*! Set DMA channel repeat count. Does not set the REPEAT bit.
 *  \param c Pointer to DMA channel for which repeat count will be set.
 *  \param count Repeat count.
 */
void dmaChSetRepeatCount(struct DMA_CH_struct *c, uint16_t count)
{
    c->REPCNT = count;
}

/*! Set DMA channel source address.
 *  \param c Pointer to DMA channel for which source address will be set.
 *  \param source Pointer to DMA channel source.
 */
void dmaChSetSource(struct DMA_CH_struct *c, void *source) {
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        c->SRCADDR0 = (uint8_t)((uint16_t)source);      // low byte of address
        c->SRCADDR1 = (uint8_t)((uint16_t)source >> 8); // high byte of address
        c->SRCADDR2 = 0;
    }
}

/*! Set DMA channel destination address.
 *  \param c Pointer to DMA channel for which destination address will be set.
 *  \param dest Pointer to DMA channel destination.
 */
void dmaChSetDestination(struct DMA_CH_struct *c, volatile void *dest) {
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        c->DESTADDR0 = (uint8_t)((uint16_t)dest);      // low byte of address
        c->DESTADDR1 = (uint8_t)((uint16_t)dest >> 8); // high byte of address
        c->DESTADDR2 = 0;
    }
}

/*! Set DMA channel transfer count.
 *  \param c Pointer to DMA channel for which transfer count will be set.
 *  \param count Transfer count.
 */
void dmaChSetTransferCount(struct DMA_CH_struct *c, uint16_t count) {
    
    /* TEMP registers are shared between ALL DMA channels */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        c->TRFCNT = count;
    }
}

/*! Get DMA channel transfer count.
 *  \param c Pointer to DMA channel for which transfer count will be returned.
 *  \return Returns value of DMA channel transfer count register.
 */
uint16_t dmaChGetTransferCount(struct DMA_CH_struct *c) {
    
    uint16_t count;
    /* TEMP registers are shared between ALL DMA channels */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count = c->TRFCNT;
    }
    return count;
}

/*! Enable DMA channel.
 *  \param c Pointer to DMA channel to enable.
 */
void dmaChEnable(struct DMA_CH_struct *c) {
    
    c->CTRLA |= DMA_CH_ENABLE_bm;
}

/*! Disable DMA channel.
 *  \param c Pointer to DMA channel to disable.
 */
void dmaChDisable(struct DMA_CH_struct *c) {
    
    c->CTRLA &= ~DMA_CH_ENABLE_bm;
}

/*! Get status of DMA channel.
 *  \return Returns 1 if channel is enabled and transaction has not yet completed, 0 if channel is
 *  not enabled.
 */
uint8_t dmaChActive(struct DMA_CH_struct *c) {
    
    return c->CTRLA & DMA_CH_ENABLE_bm;
}

