/*! \file
 *  usart.h
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Martin Clemons
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
#include <avr/pgmspace.h>
#include "task-scheduler.h"
#include "embedded-device.h"

/* Defines whether USART lib should implement interrupt handlers for USART
 * DRE/RXC/TXC interrupts or if these will be implemented by user.
 * If USART_LIB_ISR_HANDLER is defined, then DRE/RXC/TXC interrupts will be
 * handled by USART lib and `usartIsr*` functions and related will be available.
 */
#define USART_LIB_ISR_HANDLER 1


/***            Public Variables            ***/

/*! USART data order for SPI mode */
enum usartDataOrder_e {
    USART_DO_MSB_FIRST      = 0x00,             ///< Transmit byte MSB first.
    USART_DO_LSB_FIRST      = 0x01 << 2,        ///< Transmit byte LSB first.
};

/*! USART clock phase for SPI mode */
enum usartClockPhase_e {
    USART_CP_LEADING_EDGE   = 0x00,             ///< Data valid on leading edge of clock signal.
    USART_CP_TRAILING_EDGE  = 0x01 << 1,        ///< Data valid on tailing edge of clock signal.
};

/*! USART receive complete interrupt priority */
enum usartRxCompleteInterrupt_e {
    USART_RXC_INT_OFF   = USART_RXCINTLVL_OFF_gc,
    USART_RXC_INT_LO    = USART_RXCINTLVL_LO_gc,
    USART_RXC_INT_MED   = USART_RXCINTLVL_MED_gc,
    USART_RXC_INT_HI    = USART_RXCINTLVL_HI_gc,
};

/*! USART transmit complete interrupt priority */
enum usartTxCompleteInterrupt_e {
    USART_TXC_INT_OFF       = USART_TXCINTLVL_OFF_gc,
    USART_TXC_INT_LO        = USART_TXCINTLVL_LO_gc,
    USART_TXC_INT_MED       = USART_TXCINTLVL_MED_gc,
    USART_TXC_INT_HI        = USART_TXCINTLVL_HI_gc,
};

/*! USART data register empty interrupt priority */
enum usartDataRegisterEmptyInterrupt_e {
    USART_DRE_INT_OFF    = USART_DREINTLVL_OFF_gc,
    USART_DRE_INT_LO     = USART_DREINTLVL_LO_gc,
    USART_DRE_INT_MED    = USART_DREINTLVL_MED_gc,
    USART_DRE_INT_HI     = USART_DREINTLVL_HI_gc,
};

/*! A USART buffer struct, should not be modified externally. Contents
 *  subject to change.
 */
struct usartBuffer_s {
    uint8_t *b;
    uint8_t count;
    uint8_t len;
};

/*! A USART ISR IO data struct, should not be modified externally. Contents
 *  subject to change.
 */
struct usartIo_s {
    struct USART_struct *u;
    uint8_t (*txComp)(device_t *);
    uint8_t (*rxComp)(device_t *);
    void (*txcIsr)(void);
    union {
        struct {
            volatile struct usartBuffer_s tx;
            volatile struct usartBuffer_s rx;
            enum usartRxCompleteInterrupt_e rxc;
            enum usartDataRegisterEmptyInterrupt_e dre;
        } isr;
        struct {
            struct DMA_CH_struct *txD;
            struct DMA_CH_struct *rxD;
            uint8_t txBytes;
            uint8_t rxBytes;
        } dma;
    };
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
#ifdef USART_LIB_ISR_HANDLER
void usartRegisterTxcIsr(struct usartIo_s *io,
                         enum usartTxCompleteInterrupt_e txc,
                         void(*isr)(void));
void usartInitIsrIo(struct USART_struct *u, 
        struct usartIo_s *io, 
        enum usartRxCompleteInterrupt_e rxc, 
        enum usartDataRegisterEmptyInterrupt_e dre);
void usartIsrIo(struct usartIo_s *io,
        const uint8_t *tx,
        uint8_t *rx,
        uint8_t count,
        task_fp cb);
void usartIsrTx(struct usartIo_s *io,
        const uint8_t *tx,
        uint8_t count,
        task_fp cb);
void usartIsrRx(struct usartIo_s *io,
        uint8_t *rx,
        uint8_t count,
        task_fp cb);
void usartIsrRxGetBytes(struct usartIo_s *io, uint8_t *count);
#endif /* USART_LIB_ISR_HANDLER */
void usartInitDmaIo(struct USART_struct *u, 
        struct usartIo_s *io,
        struct DMA_CH_struct *txDma,
        struct DMA_CH_struct *rxDma);
void usartDmaIo(struct usartIo_s *io,
        const uint8_t *tx,
        volatile uint8_t *rx,
        uint8_t count,
        task_fp cb);
void usartDmaTx(struct usartIo_s *io,
        const uint8_t *tx,
        uint8_t count,
        task_fp cb);
void usartDmaRx(struct usartIo_s *io,
        volatile uint8_t *rx,
        uint8_t count,
        task_fp cb);

