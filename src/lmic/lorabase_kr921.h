/*
* Copyright (c) 2014-2016 IBM Corporation.
* All rights reserved.
*
* Copyright (c) 2017 MCCI Corporation
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

#ifndef _lorabase_kr921_h_
#define _lorabase_kr921_h_

#ifndef _LMIC_CONFIG_PRECONDITIONS_H_
# include "lmic_config_preconditions.h"
#endif

/****************************************************************************\
|
| Basic definitions for KR921 (always in scope)
|
\****************************************************************************/

enum _dr_kr921_t {
        KR921_DR_SF12 = 0,
        KR921_DR_SF11,
        KR921_DR_SF10,
        KR921_DR_SF9,
        KR921_DR_SF8,
        KR921_DR_SF7,
        KR921_DR_NONE
};

// Bands:
//  g1 :   1%  16dBm
//                 freq                band     datarates
enum {
        KR921_F1    = 922100000,      // g1   SF7-12
        KR921_F2    = 922300000,      // g1   SF7-12
        KR921_FDOWN = 922100000,      //      (RX2 freq, DR2)
        KR921_FBCN  = 922300000,      //      default BCN, DR3
        KR921_FPING = 922500000,      //      default ping, DR3
};
enum {
        KR921_FREQ_MIN = 920900000,
        KR921_FREQ_MAX = 923300000
};
enum {
        KR921_TX_EIRP_MAX_DBM = 16      // 16 dBm
};
enum { DR_PAGE_KR921 = 0x10 * (LMIC_REGION_kr921 - 1) };

enum { KR921_LMIC_REGION_EIRP = 1 };         // region uses EIRP

enum { KR921_LBT_US = 5000 };         // microseconds of LBT time -- 5000 ==>
					// 5 ms. We use us rather than ms for
					// future 128us support, and just for
					// backward compatibility -- there
					// is code that uses the _US constant,
					// and it's awkward to break it.

enum { KR921_LBT_DB_MAX = -80 };      // maximum channel strength in dB; if TX
					// we measure more than this, we don't tx.

// KR921 v1.1, all channels face a 1% duty cycle. So this will have to change
// in the future via a config. But this code base needs major changes for
// v1.1 in any case.
enum { KR921_V102_TX_CAP = 100 };		// v1.0.2 allows 100%

#ifndef KR921_TX_CAP
# define KR921_TX_CAP	KR921_V102_TX_CAP
#endif

#endif /* _lorabase_kr921_h_ */
