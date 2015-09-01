/*! \file
 *  device-state.c
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


#include "device-state.h"
#include "embedded-device.h"
#include <util/atomic.h>



/***                Definitions                 ***/
/*! \privatesection */



/***         Private Global Variables           ***/

static enum deviceWarningBits_e warning_bits;
static enum deviceErrorBits_e error_bits;



/***          Public Global Variables           ***/
/*! \publicsection */



/***        Private Function Prototypes         ***/
/*! \privatesection */



/***                   ISRs                     ***/



/***             Private Functions              ***/



/***             Public Functions               ***/
/*! \publicsection */

/*! Set the specified warning bits in the device warning state.
 *  \param warning Enum with warning bits to set.
 */
void deviceSetWarningState(enum deviceWarningBits_e warning)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        warning_bits |= warning;
    }
}

/*! Clear the specified warning bits in the device warning state.
 * \param warning Enum with warning bits to clear.
 */
void deviceClearWarningState(enum deviceWarningBits_e warning)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        warning_bits &= ~warning;
    }
}

/*! Clear all warning states.
 */
void deviceClearAllWarningStates(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        warning_bits = 0;
    }
}

/*! Check for a specified warning state or for a group of warning states.\n
 *  \param Enum with warning bits to check.
 *  \return Function returns 0 if warning bits are clear, non-zero if any bits
 *  specified are set. Note: It is possible to check for *any* warning bits by
 *  supplying STATE_WARNING_ALL as the function parameter.
 */
enum deviceWarningBits_e deviceCheckWarningState(enum deviceWarningBits_e warning)
{
    enum deviceWarningBits_e state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state = warning_bits & warning;
    }
    return state;
}

/*! Check for any warning states.\n
 *  \return Function returns 0 if all warning states are clear, 1 if any
 *  states are set.
 */
uint8_t deviceWarningActive(device_t *d)
{
    uint8_t state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state = warning_bits > 0;
    }
    return state;
}

/*! Set the specified error bits in the device error state.
 *  \param error Enum with error bits to set.
 */
void deviceSetErrorState(enum deviceErrorBits_e error)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        error_bits |= error;
    }
}

/*! Clear the specified error bits in the device error state.
 * \param error Enum with error bits to clear.
 */
void deviceClearErrorState(enum deviceErrorBits_e error)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        error_bits &= ~error;
    }
}

/*! Clear all error states.
 */
void deviceClearAllErrorStates(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        error_bits = 0;
    }
}

/*! Check for a specified error state or for a group of error states.\n
 *  \param Enum with error bits to check.
 *  \return Function returns 0 if error bits are clear, non-zero if any bits
 *  specified are set. Note: It is possible to check for *any* error bits by
 *  supplying STATE_error_ALL as the function parameter.
 */
enum deviceErrorBits_e deviceCheckErrorState(enum deviceErrorBits_e error)
{
    enum deviceErrorBits_e state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state = error_bits & error;
    }
    return state;
}

/*! Check for any error states.\n
 *  \return Function returns 0 if all error states are clear, 1 if any
 *  states are set.
 */
uint8_t deviceErrorActive(device_t *d)
{
    uint8_t state;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state = error_bits > 0;
    }
    return state;
}

