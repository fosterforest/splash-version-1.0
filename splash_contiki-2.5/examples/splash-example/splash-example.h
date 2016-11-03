/*
 * Copyright (c) 2011, ETH Zurich.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *
 */

/**
 * \defgroup glossy-test Simple application for testing Glossy
 * @{
 */

/**
 * \file
 *         A simple example of an application that uses Splash over Glossy, header file.
 *
 *         The application schedules Glossy periodically.
 *         The period is determined by \link GLOSSY_PERIOD \endlink.
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef GLOSSY_TEST_H_
#define GLOSSY_TEST_H_

#include "splash_plus_glossy.h"
#include "node-id.h"
#include "dev/debug_types.h"
#include <cc2420.h>

extern struct process example_collect_process;

/**
 * \defgroup glossy-test-settings Application settings
 * @{
 */

/**
 * \brief NodeId of the initiator.
 *        Default value: 200
 */
#define INITIATOR_NODE_ID 5
#define SEQ_NO_THRESHOLD_FOR_CH_SYNC 0	

/**
 * \brief Maximum number of transmissions N.
 *        Default value: 5.
 */
#define N_TX                  1 // manjunath

/**
 * \brief Period with which a Glossy phase is scheduled.
 *        Default value: 250 ms.
 */
//#define GLOSSY_PERIOD           (RTIMER_SECOND / 4)      // 250 ms 
#define GLOSSY_PERIOD           (RTIMER_SECOND/4) // manjunath

/**
 * \brief Duration of each Glossy phase.
 *        Default value: 20 ms.
 */
//#define GLOSSY_DURATION         (RTIMER_SECOND / 50)     //  20 ms
#define GLOSSY_DURATION         (RTIMER_SECOND/50) // manjunath


/**
 * \brief Guard-time at receivers.
 *        Default value: 526 us.
 */
#if COOJA
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1000)
#else
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1900)   // 526 us
#endif /* COOJA */

/**
 * \brief Number of consecutive Glossy phases with successful computation of reference time required to exit from bootstrapping.
 *        Default value: 3.
 */
#define GLOSSY_BOOTSTRAP_PERIODS 3

/**
 * \brief Period during bootstrapping at receivers.
 *        It should not be an exact fraction of \link GLOSSY_PERIOD \endlink.
 *        Default value: 69.474 ms.
 */
#define GLOSSY_INIT_PERIOD      (GLOSSY_INIT_DURATION + RTIMER_SECOND / 100)                   //  69.474 ms

/**
 * \brief Duration during bootstrapping at receivers.
 *        Default value: 59.474 ms.
 */
#define GLOSSY_INIT_DURATION    (GLOSSY_DURATION - GLOSSY_GUARD_TIME + GLOSSY_INIT_GUARD_TIME) //  59.474 ms

/**
 * \brief Guard-time during bootstrapping at receivers.
 *        Default value: 50 ms.
 */
#define GLOSSY_INIT_GUARD_TIME  (RTIMER_SECOND / 20)                                           //  50 ms

/**
 * \brief Length of data structure.
 */
//#define DATA_LEN                    sizeof(glossy_data_struct)
#define DATA_LEN 71 // manjunath
//#define DATA_LEN 30 // manjunath
//#define XOR_PATCH 1
/**
 * \brief Data structure used to represent flooding data.
 */
typedef struct {
	uint16_t parity_num_ones;
	unsigned long seq_no; /**< Sequence number, incremented by the initiator at each Glossy phase. */
	uint8_t hop;
        uint8_t dummy_data[DATA_LEN-( sizeof(unsigned long) + sizeof(uint16_t) + sizeof(uint8_t) )];
} glossy_data_struct;

/** @} */

/**
 * \defgroup glossy-test-defines Application internal defines
 * @{
 */


/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)

/**
 * \brief Check if Glossy is still bootstrapping.
 * \sa \link GLOSSY_BOOTSTRAP_PERIODS \endlink.
 */
#define GLOSSY_IS_BOOTSTRAPPING()   (skew_estimated < GLOSSY_BOOTSTRAP_PERIODS)

/**
 * \brief Check if Glossy is synchronized.
 *
 * The application assumes that a node is synchronized if it updated the reference time
 * during the last Glossy phase.
 * \sa \link is_t_ref_l_updated \endlink
 */
#define GLOSSY_IS_SYNCED()          (is_t_ref_l_updated())

/**
 * \brief Get Glossy reference time.
 * \sa \link get_t_ref_l \endlink
 */
#define GLOSSY_REFERENCE_TIME       (get_t_ref_l())

/** @} */

/** @} */

#define COLLECTION 1
#define STOP_COLLECTION 2
#define YET_TO_START_REPROGRAMMING 3
#define REPROGRAMMING 4
#define REPROGRAMMING_DONE 5
#define NUM_PROGRAM_PAGES 500
#define RECOVERY_STATE_IDLE 0
#define RECOVERY_STATE_THIRDACK_WAITING 1
#define RECOVERY_STATE_SYNACK_WAITING 2
#define RECOVERY_STATE_DATA_TX 3
#define RECOVERY_STATE_DATAPKT_WAITING 4
#define RECOVERY_STATE_DATAPKT_RECVD 5

typedef struct synack {
	uint16_t src_addr;
}synack_t;
typedef struct thirdack {
	uint16_t src_addr;
}thirdack_t;

// We need to optimize these values in the future!!!
#define SYNACK_TIMEOUT 			CLOCK_SECOND/10
#define THIRDACK_TIMEOUT 		CLOCK_SECOND/5
#define DATAPKT_TIMEOUT 		CLOCK_SECOND/5
#define INIT_TIMEOUT_FOR_SYN_TX 	1 * CLOCK_SECOND

#define GOOD_LINK_ETX_THRESHOLD 10
#define IQ_LINK_ETX_THRESHOLD 12
#define RIPPLE_DATA_PKT_MAX_RTXS 32


// define channels
#define R1_H1_CH 26 // Round1_Hop1_Channel
#define R1_H2_CH 26
#define R1_H3_CH 15
#define R1_H4_CH 15
#define R1_H5_CH 25
#define R1_H6_CH 25
#define R1_H7_CH 20
#define R1_H8_CH 20
#define R1_H9_CH 26
#define R1_H10_CH 26 

#define R2_H1_CH 15 // Round2_Hop1_Channel
#define R2_H2_CH 15
#define R2_H3_CH 26
#define R2_H4_CH 26
#define R2_H5_CH 20
#define R2_H6_CH 20
#define R2_H7_CH 25
#define R2_H8_CH 25
#define R2_H9_CH 15
#define R2_H10_CH 15 

#define R3_H1_CH 25 // Round3_Hop1_Channel
#define R3_H2_CH 25
#define R3_H3_CH 20
#define R3_H4_CH 20
#define R3_H5_CH 26
#define R3_H6_CH 26
#define R3_H7_CH 15
#define R3_H8_CH 15
#define R3_H9_CH 25
#define R3_H10_CH 25 


#endif /* GLOSSY_TEST_H_ */
