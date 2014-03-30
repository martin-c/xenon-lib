/*! \file
 *  semaphore.c
 *  xenon-lib
 *
 * Copyright (c) 2014 Martin Clemons
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
 * Semaphore locking in a single threaded application may seem useless at first,
 * however there are situations where it is nevertheless useful. Also in the case
 * of interrupt handlers Semaphores can come in handy.
 * The intent of this library is to approximate the utility provided by the Linux
 * kernel Counting Semaphore implementation. Since we're operating without a kernel however,
 * there's some limitations to what we can implement here. Basically only the core
 * sem_init(), up(), down(), and down_trylock() functions are implemented at this point.
 *
 */

#include "semaphore.h"
#include "util/atomic.h"



/***                Definitions                 ***/
/*! \privatesection */



/***         Private Global Variables           ***/



/***          Public Global Variables           ***/
/*! \publicsection */



/***        Private function prototypes         ***/
/*! \privatesection */



/***                   ISRs                     ***/



/***             Private Functions              ***/



/***             Public Functions               ***/
/*! \publicsection */

/* Initialize a semaphore.
 * \param sem Pointer to semaphore to initialize.
 * \param value initial value of semaphore.
 */
void sem_init(struct semaphore_s *sem, uint8_t value)
{
    sem->count = value;
}

/* Release a Semaphore.
 * \param sem Pointer to semaphore to release.
 */
void up(struct semaphore_s *sem)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sem->count++;
    }
}

/* Acquire a Semaphore.
 * \param sem Pointer to semaphore to acquire.
 * Function spins until semaphore is available.
 */
void down(struct semaphore_s *sem)
{
    while (sem->count == 0) {
        ;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sem->count--;
    }
}

/* Try to acquire a Semaphore.
 * \param sem Pointer to semaphore to try to acquire.
 * \return Function returns 0 if semaphore was acquired, -1 if not.
 */
int8_t down_trylock(struct semaphore_s *sem)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (sem->count == 0) {
            return -1;
        }
        sem->count--;
    }
    return 0;
}

/* Initialize a resource counter.
 * \param rsc Pointer to resource counter to initialize.
 * \param value Initial value of resource counter.
 */
void rsc_count_init(struct rsc_count_s *rsc, uint8_t value)
{
    rsc->count = value;
}

/* Increase a resource counter.
 * \param rsc Pointer to resource counter to increase.
 */
void count_up(struct rsc_count_s *rsc)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        rsc->count++;
    }
}

/* Decrease a resource counter.
 * \param rsc Pointer to resource counter to decrease.
 */
void count_down(struct rsc_count_s *rsc)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        rsc->count--;
    }
}

/* Test a resource counter for empty count.
 * \param rsc Pointer to resource counter to test.
 */
uint8_t count_empty(struct rsc_count_s *rsc)
{
    return (rsc->count == 0);
}


