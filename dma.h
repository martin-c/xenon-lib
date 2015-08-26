/*! \file
 *  dma.h
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
#include "lib-config.h"



/***                Definitions                 ***/



/***          Public Global Variables           ***/

/*! DMA Controller Configuration */
enum dmaDoubleBufferMode_e {
    DMA_DOUBLE_BUFFER_MODE_DISABLED     = DMA_DBUFMODE_DISABLED_gc, ///< no double buffer
    DMA_DOUBLE_BUFFER_MODE_CH01         = DMA_DBUFMODE_CH01_gc,     ///< double buffer on CH 0 & 1
    DMA_DOUBLE_BUFFER_MODE_CH23         = DMA_DBUFMODE_CH23_gc,     ///< double buffer on CH 2 & 3
    DMA_DOUBLE_BUFFER_MODE_CH0123       = DMA_DBUFMODE_CH01CH23_gc, ///< double buffer on CH 0,1,2,3
};

/*! DMA Channel Priority */
enum dmaChannelPriority_e {
    DMA_PRIORITY_ROUND_ROBIN    = DMA_PRIMODE_RR0123_gc,    ///< round robin (ch 0, 1, 2, 3)
    DMA_PRIORITY_CH0            = DMA_PRIMODE_CH0RR123_gc,  ///< CH0 > RR (CH1, CH2, CH3)
    DMA_PRIORITY_CH0_CH1        = DMA_PRIMODE_CH01RR23_gc,  ///< CH0 > CH1 > RR (CH 2, CH 3)
    DMA_PRIORITY_LINEAR         = DMA_PRIMODE_CH0123_gc,    ///< CH0 > CH1 > CH2 > CH3
};

/*! DMA Transaction Burst Length */
enum dmaChBurstLength_e {
    DMA_CH_BURST_LENGTH_1BYTE   = DMA_CH_BURSTLEN_1BYTE_gc, ///< 1 byte burst
    DMA_CH_BURST_LENGTH_2BYTE   = DMA_CH_BURSTLEN_2BYTE_gc, ///< 2 byte burst
    DMA_CH_BURST_LENGTH_4BYTE   = DMA_CH_BURSTLEN_4BYTE_gc, ///< 4 byte burst
    DMA_CH_BURST_LENGTH_8BYTE   = DMA_CH_BURSTLEN_8BYTE_gc, ///< 8 byte burst
};

/*! DMA Transaction Comple Interrupt Priority Level */
enum dmaChTransactionCompleteInterrupt_e {
    DMA_CH_TRX_CMP_OFF  = DMA_CH_TRNINTLVL_OFF_gc,
    DMA_CH_TRX_CMP_LO   = DMA_CH_TRNINTLVL_LO_gc,
    DMA_CH_TRX_CMP_MED  = DMA_CH_TRNINTLVL_MED_gc,
    DMA_CH_TRX_CMP_HI   = DMA_CH_TRNINTLVL_HI_gc,
};

/*! DMA channel address reload configuration */
enum dmaChAddressReload_e {
    DMA_CH_RELOAD_NONE          = 0x00,     ///< no address reload
    DMA_CH_RELOAD_BLOCK         = 0x01,     ///< reload initial address after block transfer
    DMA_CH_RELOAD_BURST         = 0x02,     ///< reload initial address after burst transfer
    DMA_CH_RELOAD_TRANSACTION   = 0x03,     ///< reload initial address after transaction completes
};

/*! DMA channel address sequencing configuration */
enum dmaChAddressMode_e {
    DMA_CH_ADDRESS_NONE         = 0x00,     ///< address fixed
    DMA_CH_ADDRESS_INC          = 0x01,     ///< increment address
    DMA_CH_ADDRESS_DEC          = 0x02,     ///< decrement address
};



/***             Public Functions               ***/

void dmaInit(enum dmaDoubleBufferMode_e dbm, enum dmaChannelPriority_e cp);
void dmaEnable(void);
void dmaDisable(void);
void dmaChInitSingleShot(struct DMA_CH_struct *c,
                         enum dmaChBurstLength_e burstLen,
                         enum dmaChAddressReload_e srcReload,
                         enum dmaChAddressMode_e srcDir,
                         enum dmaChAddressReload_e destReload,
                         enum dmaChAddressMode_e destDir,
                         enum DMA_CH_TRIGSRC_enum tSource);
#ifdef XENON_DMA_LIB_ISR_HANDLER
void dmaChRegisterIsr(struct DMA_CH_struct *c,
                      enum dmaChTransactionCompleteInterrupt_e cmpIntLvl,
                      void (*isr)(void));
#else
void dmaChSetInterruptLevel(struct DMA_CH_struct *c, enum dmaChTransactionCompleteInterrupt_e cmpIntLvl);
#endif /* XENON_DMA_LIB_ISR_HANDLER */
void dmaChEnableRepeat(struct DMA_CH_struct *c, uint8_t repCount);
void dmaChSetRepeatCount(struct DMA_CH_struct *c, uint16_t count);
void dmaChSetSource(struct DMA_CH_struct *c, void *source);
void dmaChSetDestination(struct DMA_CH_struct *c, volatile void *dest);
void dmaChSetTransferCount(struct DMA_CH_struct *c, uint16_t count);
uint16_t dmaChGetTransferCount(struct DMA_CH_struct *c);
void dmaChEnable(struct DMA_CH_struct *c);
void dmaChDisable(struct DMA_CH_struct *c);
uint8_t dmaChActive(struct DMA_CH_struct *c);



