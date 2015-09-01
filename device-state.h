/*! \file
 *  device-state.h
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


#pragma once

#include <inttypes.h>
#include <avr/io.h>
#include "embedded-device.h"



/***            Public Variables            ***/



/***            Public Functions            ***/
void deviceSetWarningState(enum deviceWarningBits_e warning);
void deviceClearWarningState(enum deviceWarningBits_e warning);
void deviceClearAllWarningStates(void);
enum deviceWarningBits_e deviceCheckWarningState(enum deviceWarningBits_e warning);
uint8_t deviceWarningActive(device_t *d);
void deviceSetErrorState(enum deviceErrorBits_e error);
void deviceClearErrorState(enum deviceErrorBits_e error);
void deviceClearAllErrorStates(void);
enum deviceErrorBits_e deviceCheckErrorState(enum deviceErrorBits_e error);
uint8_t deviceErrorActive(device_t *d);


