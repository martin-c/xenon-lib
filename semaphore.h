/*! \file
 *  semaphore.h
 *  xenon-lib
 *
 * Copyright (c) 2015 Martin Clemons
 *
 * The MIT License (MIT)
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



/***            Public Variables            ***/
struct semaphore_s {
    volatile uint8_t count;
};

struct rsc_count_s {
    volatile uint8_t count;
};



/***            Public Functions            ***/
// counting semaphore functions
void sem_init(struct semaphore_s *sem, uint8_t value);
void up(struct semaphore_s *sem);
void down(struct semaphore_s *sem);
uint8_t down_trylock(struct semaphore_s *sem);
// resource counter functions, 'inverse' semaphore
void rsc_init(struct rsc_count_s *rsc, uint8_t value);
void claim(struct rsc_count_s *rsc);
void release(struct rsc_count_s *rsc);
uint8_t rsc_released(struct rsc_count_s *rsc);

