/*! \file
 *  usart.c
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

#include "usart.h"
#include <stddef.h>



/***             Public Functions               ***/

/*! Initialize USART peripheral in Master SPI mode.
 *  \param u Pointer to USART peripheral to configure.
 *  \param order SPI data order.
 *  \param phase SPI clock phase - determines when data is valid on tx/rx pins.
 *  \param baudctrlA Baud Rate Control Register A value.
 *  \param baudctrlB Baud Rate Control Register B value.
 */
void usartInitSpi(struct USART_struct *u,
                  enum usartDataOrder_e order,
                  enum usartClockPhase_e phase,
                  uint8_t baudctrlA,
                  uint8_t baudctrlB)
{
    u->CTRLC = USART_CMODE_MSPI_gc | order | phase; // master SPI mode, set bit order and clock phase
    u->BAUDCTRLA = baudctrlA;                       // set baud rate
    u->BAUDCTRLB = baudctrlB;
    u->CTRLB = USART_RXEN_bm | USART_TXEN_bm;       // enable RX and TX
}

/*! Initialize USART peripheral in Master SPI mode, TX only.
 *  Enables only TX.
 *  \param u Pointer to USART peripheral to configure.
 *  \param order SPI data order.
 *  \param phase SPI clock phase - determines when data is valid on tx/rx pins.
 *  \param baudctrlA Baud Rate Control Register A value.
 *  \param baudctrlB Baud Rate Control Register B value.
 */
void usartInitSpiTx(struct USART_struct *u,
                    enum usartDataOrder_e order,
                    enum usartClockPhase_e phase,
                    uint8_t baudctrlA,
                    uint8_t baudctrlB)
{
    u->CTRLC = USART_CMODE_MSPI_gc | order | phase; // master SPI mode, set bit order and clock phase
    u->BAUDCTRLA = baudctrlA;                       // set baud rate
    u->BAUDCTRLB = baudctrlB;
    u->CTRLB = USART_TXEN_bm;                       // enable TX
}

/*! Initialize USART peripheral in Asynchronous communication mode.
 *  Enables TX and RX, no parity, 1 stop bit, 8 data bits per frame.
 *  \param u Pointer to USART peripheral to configure.
 *  \param baudctrlA Baud Rate Control Register A value.
 *  \param baudctrlB Baud Rate Control Register B value.
 */
void usartInitAsync(struct USART_struct *u,
                    uint8_t baudctrlA,
                    uint8_t baudctrlB)
{
    // asynchronous mode, no parity, 1 stop bits, 8 bit size
    u->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc;
    u->BAUDCTRLA = baudctrlA;                   // set baud rate
    u->BAUDCTRLB = baudctrlB;
    u->CTRLB = USART_RXEN_bm | USART_TXEN_bm;   // enable RX and TX
}

/*! Initialize USART peripheral in Asynchronous communication mode, Transmit only.
 *  Enables TX, no parity, 1 stop bit, 8 data bits per frame.
 *  \param u Pointer to USART peripheral to configure.
 *  \param baudctrlA Baud Rate Control Register A value.
 *  \param baudctrlB Baud Rate Control Register B value.
 */
void usartInitAsyncTx(struct USART_struct *u,
                      uint8_t baudctrlA,
                      uint8_t baudctrlB)
{
    // asynchronous mode, no parity, 1 stop bits, 8 bit size
    u->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc;
    u->BAUDCTRLA = baudctrlA;                   // set baud rate
    u->BAUDCTRLB = baudctrlB;
    u->CTRLB = USART_TXEN_bm;                   // enable TX only
}

/*! Flush USART data buffers.\n
 *  \param u Pointer to USART peripheral to flush.
 */
void usartFlush(struct USART_struct *u)
{
    u->DATA;
    u->DATA;
    u->DATA;
}

/*! Enable initialized but possibly disabled USART peripheral, enables tx and rx pins, flushes USART data buffer.\n
 *  \param u Pointer to USART peripheral to enable.
 */
void usartEnable(struct USART_struct *u)
{
    u->CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
    u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;   // clear interrupt and status flags
    usartFlush(u);                                  // flush data buffer
}

/*! Enable initialized but possibly disabled USART peripheral, enables tx pin only, flushes USART data buffer.\n
 *  \param u Pointer to USART peripheral to enable.
 */
void usartEnableTx(struct USART_struct *u)
{
    u->CTRLB |= USART_TXEN_bm;
    u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;   // clear interrupt and status flags
    usartFlush(u);                                  // flush data buffer
}

/*! Disable USART peripheral, disables tx and rx pins.
 *  \param u Pointer to USART peripheral to disable.
 */
void usartDisable(struct USART_struct *u)
{
    u->CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/*! Generic SPI send and receive utilizing USART peripheral in SPI mode.\n
 *  Function blocks until serial IO is complete.
 *  \param u Pointer to USART peripheral being used.
 *  \param tx Pointer to transmit buffer containing data to be sent. If 0, dummy bytes value 0 will be sent.
 *  \param rx Pointer to receive buffer where data received is written. If 0, data received is discarded.
 *  \param count Number of bytes to send/receive, maximum of 252 bytes.
 *  Since SPI sends and receives at the same time, both operations occur at once.
 */
void usartSpiIo(struct USART_struct *u, uint8_t *tx, uint8_t *rx, uint8_t count)
{
    uint8_t i = 0;      // iterate through TX buffer
    uint8_t j = 0;      // iterate through RX buffer

    u->DATA;            // clear USART rx buffer of any stale data
    u->DATA;
    u->DATA;
    u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;
    count = (count > 252) ? 252 : count;
    while (j < count) {
        // if receive buffer has data
        if (u->STATUS & USART_RXCIF_bm) {
            if (rx != NULL) {
                rx[j] = u->DATA;
            } else {
                u->DATA;                // dummy read to clear flag
            }
            j++;
        }
        /* if transmit buffer is available and bytes to send remain, write bytes to USART.
         * The reading of RXB and writing of TXB may not be atomic, so also check to make sure
         * transmit is never more than 2 bytes ahead of receive to prevent receive buffer overflows.
         */
        if (i < count && i <= j+2 && (u->STATUS & USART_DREIF_bm)) {
            // only read from tx buffer if pointer is non-null
            u->DATA = (tx != NULL) ? tx[i] : 0;
            i++;
        }
    }
}

/*! Generic SPI send and receive utilizing USART peripheral in SPI mode, reading data from program space.\n
 *  Function blocks until serial IO is complete.
 *  \param u Pointer to USART peripheral being used.
 *  \param tx Pointer to transmit buffer in program space containing data to be sent.
 *  \param rx Pointer to receive buffer where data received is written. If 0, data received is discarded.
 *  \param count Number of bytes to send/receive.
 *  Since SPI sends and receives at the same time, both operations occur at once.
 */
void usartSpiIo_P(struct USART_struct *u, const uint8_t *tx, uint8_t *rx, uint8_t count)
{
    uint8_t i = 0;      // iterate through TX buffer
    uint8_t j = 0;      // iterate through RX buffer

    u->DATA;            // clear USART rx buffer of any stale data
    u->DATA;
    u->DATA;
    u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;

    while (j < count) {
        /* if receive buffer has data */
        if (u->STATUS & USART_RXCIF_bm) {
            if (rx > 0) {
                rx[j] = u->DATA;
            } else {
                u->DATA;                // dummy read to clear flag
            }
            j++;
        }
        /* if transmit buffer is available and bytes to send remain, write bytes to USART.
         * The reading of RXB and writing of TXB may not be atomic, so also check to make sure
         * transmit is never more than 2 bytes ahead of receive to prevent receive buffer overflows.
         */
        if (i < count && i <= j+2 && (u->STATUS & USART_DREIF_bm)) {
            u->DATA = pgm_read_byte(&tx[i]);
            i++;
        }
    }
}
