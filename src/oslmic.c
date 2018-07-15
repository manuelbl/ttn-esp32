/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lmic.h"
#include "hal_esp32.h"
#include <stdbool.h>

// RUNTIME STATE
static struct {
    osjob_t* scheduledjobs;
    osjob_t* runnablejobs;
} OS;

void os_init () {
    memset(&OS, 0x00, sizeof(OS));
    hal_init();
    radio_init();
    LMIC_init();
}

ostime_t os_getTime () {
    return hal_ticks();
}

static u1_t unlinkjob (osjob_t** pnext, osjob_t* job) {
    for( ; *pnext; pnext = &((*pnext)->next)) {
        if(*pnext == job) { // unlink
            *pnext = job->next;
            return 1;
        }
    }
    return 0;
}

// clear scheduled job
void os_clearCallback (osjob_t* job) {
    hal_disableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
    u1_t res =
    #endif
    unlinkjob(&OS.scheduledjobs, job) || unlinkjob(&OS.runnablejobs, job);
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        if (res)
            lmic_printf("%lu: Cleared job %p\n", os_getTime(), job);
    #endif
}

// schedule immediately runnable job
void os_setCallback (osjob_t* job, osjobcb_t cb) {
    osjob_t** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    os_clearCallback(job);
    // fill-in job
    job->func = cb;
    job->next = NULL;
    // add to end of run queue
    for(pnext=&OS.runnablejobs; *pnext; pnext=&((*pnext)->next));
    *pnext = job;
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Scheduled job %p, cb %p ASAP\n", os_getTime(), job, cb);
    #endif
}

// schedule timed job
void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb) {
    osjob_t** pnext;
    hal_disableIRQs();
    // remove if job was already queued
    os_clearCallback(job);
    // fill-in job
    job->deadline = time;
    job->func = cb;
    job->next = NULL;
    // insert into schedule
    for(pnext=&OS.scheduledjobs; *pnext; pnext=&((*pnext)->next)) {
        if((*pnext)->deadline - time > 0) { // (cmp diff, not abs!)
            // enqueue before next element and stop
            job->next = *pnext;
            break;
        }
    }
    *pnext = job;
    hal_enableIRQs();
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Scheduled job %p, cb %p at %lu\n", os_getTime(), job, cb, time);
    #endif
}

// execute jobs from timer and from run queue
void os_runloop () {
    while(1) {
        os_runloop_once();
    }
}

void os_runloop_once() {
    #if LMIC_DEBUG_LEVEL > 1
        bool has_deadline = false;
    #endif

    osjob_t* j = NULL;
    hal_enterCriticalSection();
    // check for runnable jobs
    if(OS.runnablejobs) {
        j = OS.runnablejobs;
        OS.runnablejobs = j->next;
    } else if(OS.scheduledjobs && hal_checkTimer(OS.scheduledjobs->deadline)) { // check for expired timed jobs
        j = OS.scheduledjobs;
        OS.scheduledjobs = j->next;
    #if LMIC_DEBUG_LEVEL > 1
        has_deadline = true;
    #endif
    }
    if(j) { // run job callback
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%lu: Running job %p, cb %p, deadline %lu\n", os_getTime(), j, j->func, has_deadline ? j->deadline : 0);
    #endif
        j->func(j);
        hal_leaveCriticalSection();
    } else { // nothing pending
        hal_leaveCriticalSection();
        hal_sleep(); // wake by irq (timer already restarted)
    }
}
