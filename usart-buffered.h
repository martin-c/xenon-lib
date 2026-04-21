/*! \file
 *  usart-buffered.h
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Martin Clemons
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

/*! Interrupt-driven buffered IO for USART peripherals.
 *
 *  The hot path (byte-moving service helpers) lives in this header as
 *  `static inline __attribute__((always_inline))` functions so that the
 *  user's `ISR(...)` body inlines them directly -- no function call, only
 *  the registers the service actually touches get pushed on the ISR
 *  prologue.
 *
 *  The user owns the vectors: install a library-default ISR with the
 *  convenience macros below, or write a custom `ISR(...)` that calls the
 *  service helper (possibly after doing something else first). Vectors
 *  that are never installed don't cost any ROM.
 *
 *  Completion is exposed as a pure query (`usartTxIdle` / `usartRxIdle`).
 *  This module has no dependency on the task scheduler; an application
 *  that wants scheduler integration writes a one-line `uint8_t(device_t *)`
 *  adapter that closes over its file-static io instance.
 *
 *  Typical single-USART use:
 *      static struct usartIo_s consoleIo;
 *      USART_DEFINE_DRE_ISR(USARTC0_DRE_vect, &consoleIo)
 *      USART_DEFINE_RXC_ISR(USARTC0_RXC_vect, &consoleIo)
 *
 *      usartInitAsync(&USARTC0, BSEL_A, BSEL_B);
 *      usartBufferedIoInit(&consoleIo, &USARTC0,
 *                          USART_RXC_INT_LO, USART_DRE_INT_LO);
 *      usartBufferedRx(&consoleIo, rxBuf, sizeof rxBuf);
 */
#pragma once

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"



/***            Public Variables            ***/

/*! USART receive complete interrupt priority */
enum usartRxCompleteInterrupt_e {
    USART_RXC_INT_OFF   = USART_RXCINTLVL_OFF_gc,
    USART_RXC_INT_LO    = USART_RXCINTLVL_LO_gc,
    USART_RXC_INT_MED   = USART_RXCINTLVL_MED_gc,
    USART_RXC_INT_HI    = USART_RXCINTLVL_HI_gc,
};

/*! USART transmit complete interrupt priority */
enum usartTxCompleteInterrupt_e {
    USART_TXC_INT_OFF   = USART_TXCINTLVL_OFF_gc,
    USART_TXC_INT_LO    = USART_TXCINTLVL_LO_gc,
    USART_TXC_INT_MED   = USART_TXCINTLVL_MED_gc,
    USART_TXC_INT_HI    = USART_TXCINTLVL_HI_gc,
};

/*! USART data register empty interrupt priority */
enum usartDataRegisterEmptyInterrupt_e {
    USART_DRE_INT_OFF   = USART_DREINTLVL_OFF_gc,
    USART_DRE_INT_LO    = USART_DREINTLVL_LO_gc,
    USART_DRE_INT_MED   = USART_DREINTLVL_MED_gc,
    USART_DRE_INT_HI    = USART_DREINTLVL_HI_gc,
};

/*! USART buffer descriptor. Contents are managed by the service helpers
 *  and the `usartBuffered*` start functions; do not modify externally
 *  while a transfer is in flight.
 */
struct usartBuffer_s {
    uint8_t *b;
    uint8_t count;
    uint8_t len;
};

/*! Interrupt-driven IO state for one USART peripheral. Allocate one
 *  instance per peripheral participating in buffered IO.
 */
struct usartIo_s {
    struct USART_struct *u;
    volatile struct usartBuffer_s tx;
    volatile struct usartBuffer_s rx;
    enum usartRxCompleteInterrupt_e rxc;
    enum usartDataRegisterEmptyInterrupt_e dre;
};



/***        Inline ISR service helpers          ***/

/*! Transmit one byte from the TX buffer to the USART data register.
 *  Disables the DRE interrupt once the buffer is drained. Call from a
 *  DRE ISR or custom handler.
 */
__attribute__((always_inline))
static inline void usartTxServiceByte(struct usartIo_s *io)
{
    if (io->tx.count > 0) {
        io->u->DATA = *io->tx.b;
        io->tx.b++;
        io->tx.count--;
    }
    if (io->tx.count == 0) {
        // no more bytes to send, disable DRE interrupt
        io->u->CTRLA &= ~(USART_DREINTLVL0_bm | USART_DREINTLVL1_bm);
    }
}

/*! Receive one byte from the USART data register into the RX buffer.
 *  Disables the RXC interrupt once the buffer is full. Call from a RXC
 *  ISR or custom handler.
 */
__attribute__((always_inline))
static inline void usartRxServiceByte(struct usartIo_s *io)
{
    if (io->rx.count > 0) {
        *io->rx.b = io->u->DATA;
        io->rx.b++;
        io->rx.count--;
    }
    if (io->rx.count == 0) {
        // disable further interrupts, memory pointed to by io->rx.b
        // may no longer be valid after IO completes.
        io->u->CTRLA &= ~(USART_RXCINTLVL0_bm | USART_RXCINTLVL1_bm);
    }
}



/***        Inline completion queries           ***/

/*! Return nonzero when no bytes remain in the TX buffer (transfer is
 *  complete). Use as the condition in a scheduler-side adapter.
 */
__attribute__((always_inline))
static inline uint8_t usartTxIdle(const struct usartIo_s *io)
{
    return (io->tx.count == 0) ? 1 : 0;
}

/*! Return nonzero when no bytes remain to be received (transfer is
 *  complete). Use as the condition in a scheduler-side adapter.
 */
__attribute__((always_inline))
static inline uint8_t usartRxIdle(const struct usartIo_s *io)
{
    return (io->rx.count == 0) ? 1 : 0;
}

/*! Number of bytes received into the current RX buffer so far. Counts
 *  up from 0 toward the length passed to `usartBufferedRx()`.
 */
__attribute__((always_inline))
static inline uint8_t usartRxReceived(const struct usartIo_s *io)
{
    return io->rx.len - io->rx.count;
}



/***        ISR convenience macros              ***/

/*! Install a library-default DRE (Data Register Empty) interrupt handler
 *  for `vect` that services TX from `io_ptr`.
 *
 *  Usage (at file scope):
 *      USART_DEFINE_DRE_ISR(USARTC0_DRE_vect, &consoleIo)
 *
 *  If you need custom behavior (e.g. protocol framing), omit this macro
 *  and write your own `ISR(vect)` body, optionally calling
 *  `usartTxServiceByte(io_ptr)` to fall through to library buffering.
 */
#define USART_DEFINE_DRE_ISR(vect, io_ptr) \
    ISR(vect) { usartTxServiceByte(io_ptr); }

/*! Install a library-default RXC (Receive Complete) interrupt handler
 *  for `vect` that services RX into `io_ptr`. See `USART_DEFINE_DRE_ISR`
 *  for custom-handler guidance.
 */
#define USART_DEFINE_RXC_ISR(vect, io_ptr) \
    ISR(vect) { usartRxServiceByte(io_ptr); }

/*! Install a minimal TXC (Transmit Complete) interrupt handler for `vect`.
 *  The XMega clears TXCIF automatically on ISR entry, so the body is
 *  empty; the handler exists only to avoid the default bad-interrupt
 *  trap when the application has enabled TXCINTLVL for this USART but
 *  does not need to do any work.
 */
#define USART_DEFINE_TXC_IDLE_ISR(vect) \
    ISR(vect) { }



/***            Public Functions            ***/

void usartBufferedIoInit(struct usartIo_s *io,
                         struct USART_struct *u,
                         enum usartRxCompleteInterrupt_e rxc,
                         enum usartDataRegisterEmptyInterrupt_e dre);

void usartBufferedIo(struct usartIo_s *io,
                     const uint8_t *tx,
                     uint8_t *rx,
                     uint8_t count);

void usartBufferedTx(struct usartIo_s *io,
                     const uint8_t *tx,
                     uint8_t count);

void usartBufferedRx(struct usartIo_s *io,
                     uint8_t *rx,
                     uint8_t count);

void usartBufferedRxGetBytes(struct usartIo_s *io, uint8_t *count);
