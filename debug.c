/*! \file
 *  debug.c
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

#include "debug.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "usart.h"




/***                Definitions                 ***/



/***         Private Global Variables           ***/

/* USART assiged for debug output
 */
static struct USART_struct *debugUsart;

/* Debug output stream
 */
static FILE debugOutput;



/***          Public Global Variables           ***/

FILE *debug = &debugOutput;


/***        Private function portotypes         ***/

static int putchar_block(char c, FILE *f);
static int putchar_noblock(char c, FILE *f);



/***                   ISRs                     ***/



/***             Private Functions              ***/

/* Putchar to debug USART, blocks until USART data register is available
 */
static int putchar_block(char c, FILE *f) {
    
    // wait for buffer
    while ((debugUsart->STATUS & USART_DREIF_bm) == 0) {
        ;
    }
    // put data into buffer
    debugUsart->DATA = c;
    return 0;
}

static int putchar_noblock(char c, FILE *f) {
    
    // put data into buffer
    debugUsart->DATA = c;
    return 0;
}



/***             Public Functions               ***/

/*! Setup debugging output.
 *  This function initializes stderr stream to output on USART specified at baud rate specified.
 *  \param u Pointer to USART used for debug output
 *  \param baudctrlA Baud rate control register A
 *  \param baudctrlB Baud rate control register B
 */
void debugInit(struct USART_struct *u,
               enum debugUsartBlock_e block,
               uint8_t baudctrlA,
               uint8_t baudctrlB) {
    
    debugUsart = u;
    // initialize USART
    usartInitAsyncTx(u, baudctrlA, baudctrlB);
    // select appropriate putchar function
    if (block == DEBUG_USART_BLOCK) {
        fdev_setup_stream(debug, putchar_block, NULL,  _FDEV_SETUP_WRITE);
    } else {
        fdev_setup_stream(debug, putchar_noblock, NULL,  _FDEV_SETUP_WRITE);
    }
}


