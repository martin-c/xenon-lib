/*! \file
 *  usart-dma.h
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

/*! DMA-driven USART IO.
 *
 *  This module uses one DMA channel for TX and/or one for RX to move
 *  bytes between a user buffer and the USART DATA register. DMA trigger
 *  sources are passed explicitly at init time -- the caller already
 *  knows which USART it is configuring, so no runtime address switch is
 *  needed.
 *
 *  Completion is exposed as a pure query (`usartDmaTxIdle` /
 *  `usartDmaRxIdle`). Like `usart-buffered.h`, this module has no
 *  dependency on the task scheduler: applications wrap the completion
 *  query in their own scheduler adapter when needed.
 *
 *  Typical use:
 *      static struct usartDmaIo_s memsIo;
 *      usartInitSpi(&USARTC0, USART_DO_MSB_FIRST, ..., bA, bB);
 *      usartDmaIoInit(&memsIo, &USARTC0,
 *                     &DMA.CH0, DMA_CH_TRIGSRC_USARTC0_DRE_gc,
 *                     &DMA.CH1, DMA_CH_TRIGSRC_USARTC0_RXC_gc);
 *      usartDmaIo(&memsIo, tx, rx, 8);
 */
#pragma once

#include <inttypes.h>
#include <stddef.h>
#include <avr/io.h>
#include "usart.h"
#include "dma.h"



/***            Public Variables            ***/

/*! DMA-driven IO state for one USART peripheral. Either `txD` or `rxD`
 *  may be NULL if the corresponding direction is not used.
 */
struct usartDmaIo_s {
    struct USART_struct *u;
    struct DMA_CH_struct *txD;
    struct DMA_CH_struct *rxD;
};



/***        Inline completion queries           ***/

/*! Return nonzero when the TX DMA channel is idle (transfer complete).
 *  If no TX channel is configured, returns 1.
 */
__attribute__((always_inline))
static inline uint8_t usartDmaTxIdle(const struct usartDmaIo_s *io)
{
    if (io->txD == NULL) {
        return 1;
    }
    return (dmaChActive(io->txD) > 0) ? 0 : 1;
}

/*! Return nonzero when the RX DMA channel is idle (transfer complete).
 *  If no RX channel is configured, returns 1.
 */
__attribute__((always_inline))
static inline uint8_t usartDmaRxIdle(const struct usartDmaIo_s *io)
{
    if (io->rxD == NULL) {
        return 1;
    }
    return (dmaChActive(io->rxD) > 0) ? 0 : 1;
}



/***            Public Functions            ***/

void usartDmaIoInit(struct usartDmaIo_s *io,
                    struct USART_struct *u,
                    struct DMA_CH_struct *txDma,
                    enum DMA_CH_TRIGSRC_enum txTrigSource,
                    struct DMA_CH_struct *rxDma,
                    enum DMA_CH_TRIGSRC_enum rxTrigSource);

void usartDmaIo(struct usartDmaIo_s *io,
                const uint8_t *tx,
                volatile uint8_t *rx,
                uint8_t count);

void usartDmaTx(struct usartDmaIo_s *io,
                const uint8_t *tx,
                uint8_t count);

void usartDmaRx(struct usartDmaIo_s *io,
                volatile uint8_t *rx,
                uint8_t count);
