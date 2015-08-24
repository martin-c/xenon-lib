/*! \file
 *  usart.c
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

#include "usart.h"
#include <avr/interrupt.h>
#include <string.h>
#include <util/atomic.h>
#include "dma.h"



/***                Definitions                 ***/
/*! \privatesection */


/***         Private Global Variables           ***/



/***          Public Global Variables           ***/
/*! \publicsection */


/***        Private function prototypes         ***/
/*! \privatesection */
static inline void bufferedTx(struct usartIo_s *io);
static inline void bufferedRx(struct usartIo_s *io);



/***                   ISRs                     ***/
/* Create buffered interrupt handlers and for all existing USART peripherals.
 * Also define functions to check tx/rx status for callback handling.
 * Note: Below also defines a global usartIo_s for each USART peripheral
 * present, regardless of whether this peripheral is used for buffered IO.
 *
 * If USART_LIB_ISR_HANDLER is defined, then DRE/RXC/TXC interrupts will be
 * handled by USART lib and `usartIsr*` functions and related will be available.
 */
#if USARTC0_RXC_vect_num > 0
static struct usartIo_s *usartC0_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTC0_DRE_vect)
{
    bufferedTx(usartC0_io);
}
ISR(USARTC0_RXC_vect)
{
    bufferedRx(usartC0_io);
}
ISR(USARTC0_TXC_vect)
{
    usartC0_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartC0_txComplete(device_t *d)
{
    return (usartC0_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartC0_rxComplete(device_t *d)
{
    return (usartC0_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTC1_RXC_vect_num > 0
static struct usartIo_s *usartC1_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTC1_DRE_vect)
{
    bufferedTx(usartC1_io);
}
ISR(USARTC1_RXC_vect)
{
    bufferedRx(usartC1_io);
}
ISR(USARTC1_TXC_vect)
{
    usartC1_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartC1_txComplete(device_t *d)
{
    return (usartC1_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartC1_rxComplete(device_t *d)
{
    return (usartC1_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTD0_RXC_vect_num > 0
static struct usartIo_s *usartD0_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTD0_DRE_vect)
{
    bufferedTx(usartD0_io);
}
ISR(USARTD0_RXC_vect)
{
    bufferedRx(usartD0_io);
}
ISR(USARTD0_TXC_vect)
{
    usartD0_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartD0_txComplete(device_t *d)
{
    return (usartD0_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartD0_rxComplete(device_t *d)
{
    return (usartD0_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTD1_RXC_vect_num > 0
static struct usartIo_s *usartD1_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTD1_DRE_vect)
{
    bufferedTx(usartD1_io);
}
ISR(USARTD1_RXC_vect)
{
    bufferedRx(usartD1_io);
}
ISR(USARTD1_TXC_vect)
{
    usartD1_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartD1_txComplete(device_t *d)
{
    return (usartD1_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartD1_rxComplete(device_t *d)
{
    return (usartD1_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTE0_RXC_vect_num > 0
static struct usartIo_s *usartE0_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTE0_DRE_vect)
{
    bufferedTx(usartE0_io);
}
ISR(USARTE0_RXC_vect)
{
    bufferedRx(usartE0_io);
}
ISR(USARTE0_TXC_vect)
{
    usartE0_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartE0_txComplete(device_t *d)
{
    return (usartE0_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartE0_rxComplete(device_t *d)
{
    return (usartE0_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTE1_RXC_vect_num > 0
static struct usartIo_s *usartE1_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTE1_DRE_vect)
{
    bufferedTx(usartE1_io);
}
ISR(USARTE1_RXC_vect)
{
    bufferedRx(usartE1_io);
}
ISR(USARTE1_TXC_vect)
{
    usartE1_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartE1_txComplete(device_t *d)
{
    return (usartE1_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartE1_rxComplete(device_t *d)
{
    return (usartE1_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTF0_RXC_vect_num > 0
static struct usartIo_s *usartF0_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTF0_DRE_vect)
{
    bufferedTx(usartF0_io);
}
ISR(USARTF0_RXC_vect)
{
    bufferedRx(usartF0_io);
}
ISR(USARTF0_TXC_vect)
{
    usartF0_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartF0_txComplete(device_t *d)
{
    return (usartF0_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartF0_rxComplete(device_t *d)
{
    return (usartF0_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

#if USARTF1_RXC_vect_num > 0
static struct usartIo_s *usartF1_io;
#ifdef USART_LIB_ISR_HANDLER
ISR(USARTF1_DRE_vect)
{
    bufferedTx(usartF1_io);
}
ISR(USARTF1_RXC_vect)
{
    bufferedRx(usartF1_io);
}
ISR(USARTF1_TXC_vect)
{
    usartF1_io->txcIsr();
}
#endif /* USART_LIB_ISR_HANDLER */
static uint8_t usartF1_txComplete(device_t *d)
{
    return (usartF1_io->isr.tx.count == 0) ? 1 : 0;
}
static uint8_t usartF1_rxComplete(device_t *d)
{
    return (usartF1_io->isr.rx.count == 0) ? 1 : 0;
}
#endif

static uint8_t dmaCh0Complete(device_t *d)
{
    return (dmaChActive(&DMA.CH0) > 0) ? 0 : 1;
}
static uint8_t dmaCh1Complete(device_t *d)
{
    return (dmaChActive(&DMA.CH1) > 0) ? 0 : 1;
}
static uint8_t dmaCh2Complete(device_t *d)
{
    return (dmaChActive(&DMA.CH2) > 0) ? 0 : 1;
}
static uint8_t dmaCh3Complete(device_t *d)
{
    return (dmaChActive(&DMA.CH3) > 0) ? 0 : 1;
}



/***             Private Functions              ***/

/* Send a byte from buffer to USART data.
 */
static inline void bufferedTx(struct usartIo_s *io)
{
    if(io->isr.tx.count > 0) {
        // if bytes to send are available
        io->u->DATA = *io->isr.tx.b;
        io->isr.tx.b++;
        io->isr.tx.count--;
    }
    if (io->isr.tx.count == 0) {
        // no more bytes to send, disable DRE interrupt
        io->u->CTRLA &= ~USART_DREINTLVL_HI_gc;
    }
}

/* Receive a byte from USART data into buffer
 */
static inline void bufferedRx(struct usartIo_s *io)
{
    if(io->isr.rx.count > 0) {
        // bytes to receive
        *io->isr.rx.b = io->u->DATA;
        io->isr.rx.b++;
        io->isr.rx.count--;
    }
    if (io->isr.rx.count == 0) {
        // disable further interrupts, location pointed to by io
        // may no longer be valid after IO completes.
        io->u->CTRLA &= ~USART_RXCINTLVL_HI_gc;
    }
}



/***             Public Functions               ***/

/*! \publicsection */

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
    u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;    // clear interrupt and status flags
    usartFlush(u);                                  // flush data buffer
}

/*! Enable initialized but possibly disabled USART peripheral, enables tx pin only, flushes USART data buffer.\n
 *  \param u Pointer to USART peripheral to enable.
 */
void usartEnableTx(struct USART_struct *u)
{
    u->CTRLB |= USART_TXEN_bm;
    u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;    // clear interrupt and status flags
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
 *  Since SPI sens and receives at the same time, both operations occur at once.
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
 *  Since SPI sens and receives at the same time, both operations occur at once.
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

#ifdef USART_LIB_ISR_HANDLER
/*! Register an ISR function for the USART TX Complete interrupt. Function will
 *  be called from within the interrupt context when the USART's TXCIF is set.
 *  \param io Pointer to initialized usartIo_s.
 *  \param txc Transmit complete interrupt priority level.
 *  \param isr Pointer to ISR function.
 */
void usartRegisterTxcIsr(struct usartIo_s *io,
                         enum usartTxCompleteInterrupt_e txc,
                         void(*isr)(void))
{
    io->u->CTRLA &= ~USART_TXCINTLVL_HI_gc;     // clear previous bits
    if (isr != NULL) {
        io->u->CTRLA |= txc;
        io->txcIsr = isr;
    }
}

/*! Initialize ISR IO interface for a USART.\n
 *  This function prepares usartIo_s to be used with ISR based transmit/receive
 *  functions.
 *  \param u Pointer to initialized USART peripheral.
 *  \param io Pointer to caller-allocated usartIo_s to be initialized.
 *  \param rxc Desired receive complete interrupt priority, used for receiving bytes.
 *  \param dre Desired data register empty interrupt priority, used for sending bytes.
 */
void usartInitIsrIo(struct USART_struct *u, 
        struct usartIo_s *io, 
        enum usartRxCompleteInterrupt_e rxc, 
        enum usartDataRegisterEmptyInterrupt_e dre)
{
    if (u == NULL || io == NULL) {
        return;
    }
    // assign interrupt priority levels
    io->isr.rxc = rxc;
    io->isr.dre = dre;
    // assign USART specific data
    io->u = u;
    switch ((uint16_t)u) {
        // compare u to all existing USART peripherals
#if USARTC0_RXC_vect_num > 0
        case (uint16_t)&USARTC0:
            usartC0_io = io;
            io->txComp = usartC0_txComplete;
            io->rxComp = usartC0_rxComplete;
            break;
#endif
#if USARTC1_RXC_vect_num > 0
        case (uint16_t)&USARTC1:
            usartC1_io = io;
            io->txComp = usartC1_txComplete;
            io->rxComp = usartC1_rxComplete;
            break;
#endif
#if USARTD0_RXC_vect_num > 0
        case (uint16_t)&USARTD0:
            usartD0_io = io;
            io->txComp = usartD0_txComplete;
            io->rxComp = usartD0_rxComplete;
            break;
#endif
#if USARTD1_RXC_vect_num > 0
        case (uint16_t)&USARTD1:
            usartD1_io = io;
            io->txComp = usartD1_txComplete;
            io->rxComp = usartD1_rxComplete;
            break;
#endif
#if USARTE0_RXC_vect_num > 0
        case (uint16_t)&USARTE0:
            usartE0_io = io;
            io->txComp = usartE0_txComplete;
            io->rxComp = usartE0_rxComplete;
            break;
#endif
#if USARTE1_RXC_vect_num > 0
        case (uint16_t)&USARTE1:
            usartE1_io = io;
            io->txComp = usartE1_txComplete;
            io->rxComp = usartE1_rxComplete;
            break;
#endif
#if USARTF0_RXC_vect_num > 0
        case (uint16_t)&USARTF0:
            usartF0_io = io;
            io->txComp = usartF0_txComplete;
            io->rxComp = usartF0_rxComplete;
            break;
#endif
#if USARTF1_RXC_vect_num > 0
        case (uint16_t)&USARTF1:
            usartF1_io = io;
            io->txComp = usartF1_txComplete;
            io->rxComp = usartF1_rxComplete;
            break;
#endif           
        default:
            break;
    }
}



/*! USART send and receive utilizing USART ISR for moving data to/from buffers.
 *  \param io Pointer to initialized usartIo_s.
 *  \param tx Pointer to transmit buffer containing data to be sent.
 *  \param rx Pointer to receive buffer where data received is written.
 *  \param count Number of bytes to send/receive.
 *  \param cb Callback function to execute when RX portion of IO is complete.
 *  Note: Function call returns immediately, and USART data buffer is updated using
 *  interrupts.\n
 *  When RX is complete, cb is executed by task scheduler in normal execution context.\n
 *  Pointers io, tx, and rx must remain valid during the entire IO process and until cb is
 *  executed.
 */
void usartIsrIo(struct usartIo_s *io,
        const uint8_t *tx,
        uint8_t *rx,
        uint8_t count,
        task_fp cb)
{
    if (io == NULL || tx == NULL || rx == NULL) {
        return;
    }
    if (!io->rxComp(0) || !io->txComp(0)) {
        return;
    }
    // initialize io buffer pointers, lengths
    io->isr.tx.b = (uint8_t *)tx;
    io->isr.tx.count = count;
    io->isr.tx.len = count;
    io->isr.rx.b = rx;
    io->isr.rx.count = count;
    io->isr.rx.len = count;
    // initialize callback, executed when RX is complete
    if (cb != NULL) {
        tsNewConditionalSingleShotTask(cb, io->rxComp);
    }
    // initialize interrupts
    io->u->STATUS |= USART_RXCIF_bm | USART_TXCIF_bm;
    io->u->CTRLA = io->isr.rxc | io->isr.dre;
}

/*! USART transmit utilizing USART ISR for moving data from buffer.
 *  \param io Pointer to initialized usartIo_s.
 *  \param tx Pointer to transmit buffer containing data to be sent.
 *  \param count Number of bytes to send.
 *  \param cb Callback function to execute when TX is complete.
 *  Note: Function call returns immediately, and USART data buffer is updated using
 *  DRE interrupt.\n
 *  When TX is complete, cb is executed by task scheduler in normal execution context.\n
 *  Pointers io and tx must remain valid during the entire IO process and until cb is
 *  executed.
 */
void usartIsrTx(struct usartIo_s *io,
        const uint8_t *tx,
        uint8_t count,
        task_fp cb)
{
    if (io == NULL || tx == NULL) {
        return;
    }
    if (!io->txComp(0)) {
        return;
    }
    // initialize io buffer pointers, lengths
    io->isr.tx.b = (uint8_t *)tx;
    io->isr.tx.count = count;
    io->isr.tx.len = count;
    // initialize callback, executed when TX is complete
    if (cb != NULL) {
        tsNewConditionalSingleShotTask(cb, io->txComp);
    }
    // initialize interrupts
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // update atomically to prevent clobbering RX routine ISR state
        io->u->CTRLA |= io->isr.dre;
        io->u->STATUS |= USART_TXCIF_bm;
    }
}

/*! USART receive utilizing USART ISR for moving data to buffer.
 *  \param io Pointer to initialized usartIo_s.
 *  \param rx Pointer to receive buffer where data received is written.
 *  \param count Number of bytes to receive.
 *  \param cb Callback function to execute when RX is complete.
 *  Note: Function call returns immediately, and USART data buffer is updated using
 *  interrupts.\n
 *  When IO is complete, cb is executed by task scheduler in normal execution context.\n
 *  Pointers io and rx must remain valid during the entire IO process and until cb is
 *  executed.
 */
void usartIsrRx(struct usartIo_s *io,
        uint8_t *rx,
        uint8_t count,
        task_fp cb)
{
    if (io == NULL || rx == NULL) {
        return;
    }
    if (!io->rxComp(0)) {
        return;
    }
    // initialize io buffer pointers, lengths
    io->isr.rx.b = rx;
    io->isr.rx.count = count;
    io->isr.rx.len = count;
    // initialize callback, executed when RX is complete
    if (cb != NULL) {
        tsNewConditionalSingleShotTask(cb, io->rxComp);
    }
    // initialize interrupts
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // update atomically to prevent clobbering TX routine ISR state
        io->u->CTRLA |= io->isr.rxc;
        io->u->STATUS |= USART_RXCIF_bm;
    }
}

/*! Get byte count received by previously scheduled ISR IO. Cancel existing ISR
 *  receive, enable RX complete callback.
 *  \param io Pointer to initialized usartIo_s currently being used for receive.
 *  \param count Bytes received thus far is written here.
 *  Note: Function sets internal states such that task scheduler will execute
 *  any registered RX complete callback during next scheduler check.
 */
void usartIsrRxGetBytes(struct usartIo_s *io, uint8_t *count)
{
    if (io == NULL || count == NULL) {
        return;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // calculate bytes received
        *count = io->isr.rx.len - io->isr.rx.count;
        // terminate rx
        io->isr.rx.count = 0;
        io->u->CTRLA &= ~USART_RXCINTLVL_HI_gc;
    }
    /* Since rx.count has been set to zero, the task scheduler will execute the RX complete callback now.
     */
}

#endif /* USART_LIB_ISR_HANDLER */

/*! Initialize DMA IO interface for a USART.\n
 *  This function prepares usartIo_s to be used with DMA based transmit/receive
 *  functions.
 *  \param u Pointer to initialized USART peripheral.
 *  \param io Pointer to caller-allocated usartIo_s to be initialized.
 *  \param txDma Pointer to DMA channel used for transmit.
 *  \param rxDma Pointer to DMA channel used for receive.
 *  Note: Either DMA channel pointer may be NULL if only transmit or receive capability
 *  is desired.
 */
void usartInitDmaIo(struct USART_struct *u, 
        struct usartIo_s *io,
        struct DMA_CH_struct *txDma,
        struct DMA_CH_struct *rxDma)
{
    if (u == NULL || io == NULL) {
        return;
    }
    enum DMA_CH_TRIGSRC_enum txTrigSource = 0;
    enum DMA_CH_TRIGSRC_enum rxTrigSource = 0;
    // assign USART specific data
    io->u = u;
    switch ((uint16_t)u) {
        // compare u to all existing USART peripherals
#if USARTC0_RXC_vect_num > 0
        case (uint16_t)&USARTC0:
            usartC0_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTC0_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTC0_RXC_gc;
            break;
#endif
#if USARTC1_RXC_vect_num > 0
        case (uint16_t)&USARTC1:
            usartC1_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTC1_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTC1_RXC_gc;
            break;
#endif
#if USARTD0_RXC_vect_num > 0
        case (uint16_t)&USARTD0:
            usartD0_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTD0_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTD0_RXC_gc;
            break;
#endif
#if USARTD1_RXC_vect_num > 0
        case (uint16_t)&USARTD1:
            usartD1_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTD1_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTD1_RXC_gc;
            break;
#endif
#if USARTE0_RXC_vect_num > 0
        case (uint16_t)&USARTE0:
            usartE0_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTE0_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTE0_RXC_gc;
            break;
#endif
#if USARTE1_RXC_vect_num > 0
        case (uint16_t)&USARTE1:
            usartE1_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTE1_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTE1_RXC_gc;
            break;
#endif
#if USARTF0_RXC_vect_num > 0
        case (uint16_t)&USARTF0:
            usartF0_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTF0_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTF0_RXC_gc;
            break;
#endif
#if USARTF1_RXC_vect_num > 0
        case (uint16_t)&USARTF1:
            usartF1_io = io;
            txTrigSource = DMA_CH_TRIGSRC_USARTF1_DRE_gc;
            rxTrigSource = DMA_CH_TRIGSRC_USARTF1_RXC_gc;
            break;
#endif           
        default:
            break;
    }
    
    // initialize DMA channels
    if (txDma != NULL) {
        dmaChInitSingleShot(txDma,
                DMA_CH_BURST_LENGTH_1BYTE,
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_INC,
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_NONE,
                txTrigSource);
        dmaChSetDestination(txDma, &u->DATA);
        switch ((uint16_t)txDma) {
            case (uint16_t)&DMA.CH0:
                io->txComp = dmaCh0Complete;
                break;
            case (uint16_t)&DMA.CH1:
                io->txComp = dmaCh1Complete;
                break;
            case (uint16_t)&DMA.CH2:
                io->txComp = dmaCh2Complete;
                break;
            case (uint16_t)&DMA.CH3:
                io->txComp = dmaCh3Complete;
                break;
        }
    }
    if (rxDma != NULL) {
        dmaChInitSingleShot(rxDma,
                DMA_CH_BURST_LENGTH_1BYTE,
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_NONE,
                DMA_CH_RELOAD_NONE,
                DMA_CH_ADDRESS_INC,
                rxTrigSource);
        dmaChSetSource(rxDma, (void *)&u->DATA);
        switch ((uint16_t)rxDma) {
            case (uint16_t)&DMA.CH0:
                io->rxComp = dmaCh0Complete;
                break;
            case (uint16_t)&DMA.CH1:
                io->rxComp = dmaCh1Complete;
                break;
            case (uint16_t)&DMA.CH2:
                io->rxComp = dmaCh2Complete;
                break;
            case (uint16_t)&DMA.CH3:
                io->rxComp = dmaCh3Complete;
                break;
        }
    }
    io->dma.txD = txDma;
    io->dma.rxD = rxDma;
}

/*! USART send and receive utilizing DMA for moving data to/from buffers.
 *  \param io Pointer to initialized usartIo_s.
 *  \param tx Pointer to transmit buffer containing data to be sent.
 *  \param rx Pointer to receive buffer where data received is written.
 *  \param count Number of bytes to send/receive.
 *  \param cb Callback function to execute when RX portion of IO is complete.
 *  Note: Function call returns immediately, and USART data buffer is updated using
 *  DMA.\n
 *  When RX is complete, cb is executed by task scheduler in normal execution context.\n
 *  Pointers io, tx, and rx must remain valid during the entire IO process and until cb is
 *  executed.
 */
void usartDmaIo(struct usartIo_s *io,
        const uint8_t *tx,
        volatile uint8_t *rx,
        uint8_t count,
        task_fp cb)
{
    if (io == NULL || tx == NULL || rx == NULL) {
        return;
    }
    if (!io->rxComp(0) || !io->txComp(0)) {
        return;
    }
    // initialize io buffer pointers, lengths
    dmaChSetSource(io->dma.txD, (void *)tx);
    dmaChSetTransferCount(io->dma.txD, count);
    dmaChSetDestination(io->dma.rxD, rx);
    dmaChSetTransferCount(io->dma.rxD, count);
    // initialize callback, executed when RX is complete
    if (cb != NULL) {
        tsNewConditionalSingleShotTask(cb, io->rxComp);
    }
    // start DMA transfers
    dmaChEnable(io->dma.rxD);
    dmaChEnable(io->dma.txD);
}

/*! USART send utilizing DMA for moving data from buffer.
 *  \param io Pointer to initialized usartIo_s.
 *  \param tx Pointer to transmit buffer containing data to be sent.
 *  \param count Number of bytes to send.
 *  \param cb Callback function to execute when IO is complete.
 *  Note: Function call returns immediately, and USART data buffer is updated using
 *  DMA.\n
 *  When TX is complete, cb is executed by task scheduler in normal execution context.\n
 *  Pointers io, and tx must remain valid during the entire IO process and until cb is
 *  executed.
 */
void usartDmaTx(struct usartIo_s *io,
        const uint8_t *tx,
        uint8_t count,
        task_fp cb)
{
    if (io == NULL || tx == NULL) {
        return;
    }
    if (io->txComp(0) == 0) {
        return;
    }
    // initialize io buffer pointers, lengths
    dmaChSetSource(io->dma.txD, (void *)tx);
    dmaChSetTransferCount(io->dma.txD, count);
    // initialize callback, executed when RX is complete
    if (cb != NULL) {
        tsNewConditionalSingleShotTask(cb, io->txComp);
    }
    // start DMA transfers
    dmaChEnable(io->dma.txD);
}

/*! USART receive utilizing DMA for moving data to buffer.
 *  \param io Pointer to initialized usartIo_s.
 *  \param rx Pointer to receive buffer where data received is written.
 *  \param count Number of bytes to receive.
 *  \param cb Callback function to execute when IO is complete.
 *  Note: Function call returns immediately, and USART data buffer is updated using
 *  DMA.\n
 *  When RX is complete, cb is executed by task scheduler in normal execution context.\n
 *  Pointers io and rx must remain valid during the entire IO process and until cb is
 *  executed.
 */
void usartDmaRx(struct usartIo_s *io,
        volatile uint8_t *rx,
        uint8_t count,
        task_fp cb)
{
    if (io == NULL || rx == NULL) {
        return;
    }
    if (!io->rxComp(0)) {
        return;
    }
    // initialize io buffer pointers, lengths
    dmaChSetDestination(io->dma.rxD, rx);
    dmaChSetTransferCount(io->dma.rxD, count);
    // initialize callback, executed when RX is complete
    if (cb != NULL) {
        tsNewConditionalSingleShotTask(cb, io->rxComp);
    }
    // start DMA transfers
    dmaChEnable(io->dma.rxD);
}

