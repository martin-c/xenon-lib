/*! \file
 *  usart-buffered.c
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

#include "usart-buffered.h"
#include <stddef.h>
#include <util/atomic.h>



/***             Public Functions               ***/

/*! Initialize ISR-driven IO state for a USART.
 *  Records the USART pointer and the interrupt priority levels to be used
 *  when starting transfers. The USART itself must already be configured
 *  (e.g. via `usartInitAsync`) and global interrupts must be enabled at
 *  the requested priority levels in PMIC for buffered IO to proceed.
 *
 *  \param io  Caller-allocated state struct to initialize.
 *  \param u   Pointer to configured USART peripheral.
 *  \param rxc Desired RX complete interrupt priority, used when receiving.
 *  \param dre Desired DRE interrupt priority, used when transmitting.
 */
void usartBufferedIoInit(struct usartIo_s *io,
                         struct USART_struct *u,
                         enum usartRxCompleteInterrupt_e rxc,
                         enum usartDataRegisterEmptyInterrupt_e dre)
{
    if (io == NULL || u == NULL) {
        return;
    }
    io->u = u;
    io->rxc = rxc;
    io->dre = dre;
    io->tx.b = NULL;
    io->tx.count = 0;
    io->tx.len = 0;
    io->rx.b = NULL;
    io->rx.count = 0;
    io->rx.len = 0;
}

/*! Start a simultaneous send/receive through the USART ISR.
 *  Returns immediately; the transfer completes via the DRE and RXC
 *  interrupt handlers installed for this USART. Use `usartRxIdle(io)`
 *  (typically from a scheduler adapter) to detect completion.
 *
 *  The caller must keep `tx` and `rx` valid until the transfer is done.
 *  Previously scheduled IO on the same `io` must be complete; this
 *  function is a no-op otherwise.
 *
 *  \param io    Initialized IO state.
 *  \param tx    Transmit buffer.
 *  \param rx    Receive buffer.
 *  \param count Number of bytes to send and receive.
 */
void usartBufferedIo(struct usartIo_s *io,
                     const uint8_t *tx,
                     uint8_t *rx,
                     uint8_t count)
{
    if (io == NULL || tx == NULL || rx == NULL) {
        return;
    }
    if (!usartTxIdle(io) || !usartRxIdle(io)) {
        return;
    }
    io->tx.b = (uint8_t *)tx;
    io->tx.count = count;
    io->tx.len = count;
    io->rx.b = rx;
    io->rx.count = count;
    io->rx.len = count;
    io->u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;
    io->u->CTRLA = io->rxc | io->dre;
}

/*! Start a transmit through the USART DRE ISR.
 *  Returns immediately; the TX buffer is drained in the DRE handler.
 *  Use `usartTxIdle(io)` to detect completion.
 *
 *  Updates `CTRLA` inside `ATOMIC_BLOCK` so that a concurrent RX
 *  routine running at a different interrupt priority does not clobber
 *  the RXC configuration bits.
 *
 *  \param io    Initialized IO state.
 *  \param tx    Transmit buffer; must remain valid until TX completes.
 *  \param count Number of bytes to send.
 */
void usartBufferedTx(struct usartIo_s *io,
                     const uint8_t *tx,
                     uint8_t count)
{
    if (io == NULL || tx == NULL) {
        return;
    }
    if (!usartTxIdle(io)) {
        return;
    }
    io->tx.b = (uint8_t *)tx;
    io->tx.count = count;
    io->tx.len = count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        io->u->CTRLA |= io->dre;
        io->u->STATUS |= USART_TXCIF_bm;
    }
}

/*! Start a receive through the USART RXC ISR.
 *  Returns immediately; data is written into `rx` as the RXC handler
 *  fires. Use `usartRxIdle(io)` to detect completion.
 *
 *  Does not clear the RXCIF flag: if bytes have already accumulated in
 *  the USART receive buffer they will be serviced as soon as the
 *  interrupt is enabled.
 *
 *  \param io    Initialized IO state.
 *  \param rx    Receive buffer; must remain valid until RX completes.
 *  \param count Number of bytes to receive.
 */
void usartBufferedRx(struct usartIo_s *io,
                     uint8_t *rx,
                     uint8_t count)
{
    if (io == NULL || rx == NULL) {
        return;
    }
    if (!usartRxIdle(io)) {
        return;
    }
    io->rx.b = rx;
    io->rx.count = count;
    io->rx.len = count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        io->u->CTRLA |= io->rxc;
    }
}

/*! Report bytes received so far and terminate the pending receive.
 *  Disables the RXC interrupt and sets `rx.count` to zero so the next
 *  call to `usartRxIdle` returns true. The scheduler adapter watching
 *  that predicate will then fire its callback.
 *
 *  \param io    IO state currently driving a receive.
 *  \param count Set to the number of bytes received before termination.
 */
void usartBufferedRxGetBytes(struct usartIo_s *io, uint8_t *count)
{
    if (io == NULL || count == NULL) {
        return;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *count = io->rx.len - io->rx.count;
        io->rx.count = 0;
        io->u->CTRLA &= ~USART_RXCINTLVL_gm;
    }
}
