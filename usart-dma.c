/*! \file
 *  usart-dma.c
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

#include "usart-dma.h"
#include <stddef.h>



/***             Public Functions               ***/

/*! Initialize DMA-driven IO state for a USART.
 *  Sets up the configured DMA channel(s) as single-shot 1-byte-burst
 *  transfers to/from the USART DATA register, triggered by the
 *  specified USART DMA trigger source. Either direction may be omitted
 *  by passing NULL for the channel pointer (the trigger source for that
 *  direction is then ignored).
 *
 *  \param io             Caller-allocated state struct to initialize.
 *  \param u              Pointer to configured USART peripheral.
 *  \param txDma          DMA channel used for transmit, or NULL.
 *  \param txTrigSource   USART DRE trigger source for txDma (e.g.
 *                        `DMA_CH_TRIGSRC_USARTC0_DRE_gc`).
 *  \param rxDma          DMA channel used for receive, or NULL.
 *  \param rxTrigSource   USART RXC trigger source for rxDma (e.g.
 *                        `DMA_CH_TRIGSRC_USARTC0_RXC_gc`).
 */
void usartDmaIoInit(struct usartDmaIo_s *io,
                    struct USART_struct *u,
                    struct DMA_CH_struct *txDma,
                    enum DMA_CH_TRIGSRC_enum txTrigSource,
                    struct DMA_CH_struct *rxDma,
                    enum DMA_CH_TRIGSRC_enum rxTrigSource)
{
    if (io == NULL || u == NULL) {
        return;
    }
    io->u = u;
    io->txD = txDma;
    io->rxD = rxDma;

    if (txDma != NULL) {
        dmaChInitSingleShot(txDma,
                DMA_CH_BURST_LENGTH_1BYTE,
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_INC,       // source: user buffer, incrementing
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_NONE,      // dest: USART DATA, fixed
                txTrigSource);
        dmaChSetDestination(txDma, &u->DATA);
    }
    if (rxDma != NULL) {
        dmaChInitSingleShot(rxDma,
                DMA_CH_BURST_LENGTH_1BYTE,
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_NONE,      // source: USART DATA, fixed
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_INC,       // dest: user buffer, incrementing
                rxTrigSource);
        dmaChSetSource(rxDma, (void *)&u->DATA);
    }
}

/*! Start a simultaneous DMA send and receive.
 *  Returns immediately; the DMA controller moves bytes to/from the USART
 *  DATA register as the peripheral signals DRE/RXC. Use
 *  `usartDmaRxIdle(io)` to detect completion.
 *
 *  Both DMA channels must have been configured via `usartDmaIoInit` and
 *  any previously started transfer on the same channels must be idle.
 *
 *  \param io    Initialized IO state (both txD and rxD required).
 *  \param tx    Transmit buffer.
 *  \param rx    Receive buffer.
 *  \param count Number of bytes.
 */
void usartDmaIo(struct usartDmaIo_s *io,
                const uint8_t *tx,
                volatile uint8_t *rx,
                uint8_t count)
{
    if (io == NULL || tx == NULL || rx == NULL) {
        return;
    }
    if (io->txD == NULL || io->rxD == NULL) {
        return;
    }
    if (!usartDmaTxIdle(io) || !usartDmaRxIdle(io)) {
        return;
    }
    dmaChSetSource(io->txD, (void *)tx);
    dmaChSetTransferCount(io->txD, count);
    dmaChSetDestination(io->rxD, rx);
    dmaChSetTransferCount(io->rxD, count);
    dmaChEnable(io->rxD);
    dmaChEnable(io->txD);
}

/*! Start a DMA transmit.
 *  Returns immediately; the TX DMA channel drains the buffer to the
 *  USART as DRE triggers fire. Use `usartDmaTxIdle(io)` to detect
 *  completion.
 *
 *  \param io    Initialized IO state (txD required).
 *  \param tx    Transmit buffer; must remain valid until TX completes.
 *  \param count Number of bytes.
 */
void usartDmaTx(struct usartDmaIo_s *io,
                const uint8_t *tx,
                uint8_t count)
{
    if (io == NULL || tx == NULL || io->txD == NULL) {
        return;
    }
    if (!usartDmaTxIdle(io)) {
        return;
    }
    dmaChSetSource(io->txD, (void *)tx);
    dmaChSetTransferCount(io->txD, count);
    dmaChEnable(io->txD);
}

/*! Start a DMA receive.
 *  Returns immediately; data is written into `rx` as the RX DMA channel
 *  services RXC triggers. Use `usartDmaRxIdle(io)` to detect completion.
 *
 *  \param io    Initialized IO state (rxD required).
 *  \param rx    Receive buffer; must remain valid until RX completes.
 *  \param count Number of bytes.
 */
void usartDmaRx(struct usartDmaIo_s *io,
                volatile uint8_t *rx,
                uint8_t count)
{
    if (io == NULL || rx == NULL || io->rxD == NULL) {
        return;
    }
    if (!usartDmaRxIdle(io)) {
        return;
    }
    dmaChSetDestination(io->rxD, rx);
    dmaChSetTransferCount(io->rxD, count);
    dmaChEnable(io->rxD);
}
