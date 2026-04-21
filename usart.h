/*! \file
 *  usart.h
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

/*! Core USART peripheral configuration and blocking IO.
 *
 *  This header covers the always-needed pieces: peripheral init
 *  (async and SPI modes), enable/disable, flush, and blocking
 *  SPI master IO. It has no dependency on the task scheduler or
 *  any particular IO strategy.
 *
 *  For interrupt-driven buffered IO include `usart-buffered.h`.
 *  For DMA-driven IO include `usart-dma.h`. Both are independent
 *  of this file's blocking API and of each other.
 */
#pragma once

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>



/***            Public Variables            ***/

/*! USART data order for SPI mode */
enum usartDataOrder_e {
    USART_DO_MSB_FIRST      = 0x00,             ///< Transmit byte MSB first.
    USART_DO_LSB_FIRST      = 0x01 << 2,        ///< Transmit byte LSB first.
};

/*! USART clock phase for SPI mode */
enum usartClockPhase_e {
    USART_CP_LEADING_EDGE   = 0x00,             ///< Data valid on leading edge of clock signal.
    USART_CP_TRAILING_EDGE  = 0x01 << 1,        ///< Data valid on trailing edge of clock signal.
};



/***            Public Functions            ***/

void usartInitSpi(struct USART_struct *u,
                  enum usartDataOrder_e order,
                  enum usartClockPhase_e phase,
                  uint8_t baudctrlA,
                  uint8_t baudctrlB);

void usartInitSpiTx(struct USART_struct *u,
                    enum usartDataOrder_e order,
                    enum usartClockPhase_e phase,
                    uint8_t baudctrlA,
                    uint8_t baudctrlB);

void usartInitAsync(struct USART_struct *u,
                    uint8_t baudctrlA,
                    uint8_t baudctrlB);

void usartInitAsyncTx(struct USART_struct *u,
                      uint8_t baudctrlA,
                      uint8_t baudctrlB);

void usartFlush(struct USART_struct *u);
void usartEnable(struct USART_struct *u);
void usartEnableTx(struct USART_struct *u);
void usartDisable(struct USART_struct *u);
void usartSpiIo(struct USART_struct *u, uint8_t *tx, uint8_t *rx, uint8_t count);
void usartSpiIo_P(struct USART_struct *u, const uint8_t *tx, uint8_t *rx, uint8_t count);
