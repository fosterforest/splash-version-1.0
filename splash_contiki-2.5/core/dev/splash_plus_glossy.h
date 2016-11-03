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
 * \file
 *         Glossy + Splash core, header file.
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *	   Splash: Manjunath D <doddaven@comp.nus.edu.sg>
 */

#ifndef GLOSSY_H_
#define GLOSSY_H_

#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/cc2420_const.h"
#include "dev/cc2420.h"
#include "dev/leds.h"
#include "dev/splash_plus_glossy_spi.h"
#include "dev/xmem.h"
#include <io.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include "splash-example.h"

/**
 * If not zero, nodes print additional debug information (disabled by default).
 */
#define GLOSSY_DEBUG 0
/**
 * Size of the window used to average estimations of slot lengths.
 */
#define GLOSSY_SYNC_WINDOW            64
/**
 * If the relay counter has this or a higher value, do not use it
 * for synchronization.
 */
#define MAX_VALID_RELAY_CNT           200
/**
 * Initiator timeout, in DCO clock ticks.
 * When the timeout expires, if the initiator has not received any packet
 * after its first transmission it transmits again.
 */
#define GLOSSY_INITIATOR_TIMEOUT      (3 * T_slot_h)

/**
 * Ratio between the frequencies of the DCO and the low-frequency clocks
 */
#if COOJA
#define CLOCK_PHI                     (4194304uL / RTIMER_SECOND)
#else
#define CLOCK_PHI                     (F_CPU / RTIMER_SECOND)
#endif /* COOJA */

#define GLOSSY_HEADER                 0xfe
#define OAP_SECOND_ROUND_HEADER       35	//0xfd
#define CONTROL_MSG_HEADER	      170 	//0xfc
#define GLOSSY_HEADER_LEN             sizeof(uint8_t)
#define GLOSSY_RELAY_CNT_LEN          sizeof(uint8_t)
#define GLOSSY_IS_ON()                (get_state() != GLOSSY_STATE_OFF)
#define FOOTER_LEN                    2
#define FOOTER1_CRC_OK                0x80
#define FOOTER1_CORRELATION           0x7f

#define GLOSSY_LEN_FIELD              packet[0]
#define GLOSSY_HEADER_FIELD           packet[1]
#define GLOSSY_DATA_FIELD             packet[2]
#define GLOSSY_DATA_SRC		      packet[6]
#define GLOSSY_DATA_CHANNEL	      packet[7]
#define GLOSSY_RELAY_CNT_FIELD        packet[packet_len - FOOTER_LEN] // last byte in the data field
#define GLOSSY_RSSI_FIELD             packet[packet_len - 1]
#define GLOSSY_CRC_FIELD              packet[packet_len]

enum {
	GLOSSY_INITIATOR = 1, GLOSSY_RECEIVER = 0
};

enum {
	GLOSSY_SYNC = 1, GLOSSY_NO_SYNC = 0
};
/**
 * List of possible Glossy states.
 */
enum glossy_state {
	GLOSSY_STATE_OFF,          /**< Glossy is not executing */
	GLOSSY_STATE_WAITING,      /**< Glossy is waiting for a packet being flooded */
	GLOSSY_STATE_RECEIVING,    /**< Glossy is receiving a packet */
	GLOSSY_STATE_RECEIVED,     /**< Glossy has just finished receiving a packet */
	GLOSSY_STATE_TRANSMITTING, /**< Glossy is transmitting a packet */
	GLOSSY_STATE_TRANSMITTED,  /**< Glossy has just finished transmitting a packet */
	GLOSSY_STATE_ABORTED,       /**< Glossy has just aborted a packet reception */
	GLOSSY_READING_XMEM
};
#if GLOSSY_DEBUG
unsigned int high_T_irq, rx_timeout, bad_length, bad_header, bad_crc;
#endif /* GLOSSY_DEBUG */

PROCESS_NAME(glossy_process);

/* ----------------------- Application interface -------------------- */
/**
 * \defgroup glossy_interface Glossy API
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

/**
 * \defgroup glossy_main Interface related to flooding
 * @{
 */

/**
 * \brief            Start Glossy and stall all other application tasks.
 *
 * \param data_      A pointer to the flooding data.
 *
 *                   At the initiator, Glossy reads from the given memory
 *                   location data provided by the application.
 *
 *                   At a receiver, Glossy writes to the given memory
 *                   location data for the application.
 * \param data_len_  Length of the flooding data, in bytes.
 * \param initiator_ Not zero if the node is the initiator,
 *                   zero if it is a receiver.
 * \param sync_      Not zero if Glossy must provide time synchronization,
 *                   zero otherwise.
 * \param tx_max_    Maximum number of transmissions (N).
 */
void splash_start(uint8_t *data_, uint8_t data_len_, uint8_t initiator_, uint8_t sync_, uint8_t tx_max_, uint16_t hopcount, uint8_t num_children);

/**
 * \brief            Stop Glossy and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 * \sa               get_rx_cnt
 */
uint16_t splash_stop(uint8_t end);

/**
 * \brief            Get the last received counter.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint16_t get_rx_cnt(void);
char tx_timeout_hanlder(struct rtimer *t, void *ptr);

// manjunath: my functionS
int8_t get_rssi_val(void);
uint8_t get_tx_cnt(void);
uint8_t get_packet_len(void);
unsigned long get_seq_num(void);
uint8_t get_src_addr(void);
unsigned int get_bad_pkts_cnt(void);
uint8_t *get_paket();

/**
 * \brief            Get the current Glossy state.
 * \return           Current Glossy state, one of the possible values
 *                   of \link glossy_state \endlink.
 */
uint8_t get_state(void);

/**
 * \brief            Get low-frequency time of first packet reception
 *                   during the last Glossy phase.
 * \returns          Low-frequency time of first packet reception
 *                   during the last Glossy phase.
 */
rtimer_clock_t get_t_first_rx_l(void);

/** @} */

/**
 * \defgroup glossy_sync Interface related to time synchronization
 * @{
 */

/**
 * \brief            Get the last relay counter.
 * \returns          Value of the relay counter embedded in the first packet
 *                   received during the last Glossy phase.
 */
uint8_t get_relay_cnt(void);

/**
 * \brief            Get the local estimation of T_slot, in DCO clock ticks.
 * \returns          Local estimation of T_slot.
 */
rtimer_clock_t get_T_slot_h(void);

/**
 * \brief            Get low-frequency synchronization reference time.
 * \returns          Low-frequency reference time
 *                   (i.e., time at which the initiator started the flood).
 */
rtimer_clock_t get_t_ref_l(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Glossy phase, zero otherwise.
 */
uint8_t is_t_ref_l_updated(void);

/**
 * \brief            Set low-frequency synchronization reference time.
 * \param t          Updated reference time.
 *                   Useful to manually update the reference time if a
 *                   packet has not been received.
 */
void set_t_ref_l(rtimer_clock_t t);

/**
 * \brief            Set the current synchronization status.
 * \param updated    Not zero if a node has to be considered synchronized,
 *                   zero otherwise.
 */
void set_t_ref_l_updated(uint8_t updated);

/** @} */

/** @} */

/**
 * \defgroup glossy_internal Glossy internal functions
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

/* ------------------------------ Timeouts -------------------------- */
/**
 * \defgroup glossy_timeouts Timeouts
 * @{
 */

inline void glossy_schedule_rx_timeout(void);
inline void glossy_stop_rx_timeout(void);
inline void glossy_schedule_initiator_timeout(void);
inline void glossy_stop_initiator_timeout(void);
inline void glossy_schedule_tx_timeout(uint8_t pkt_type);
//inline void glossy_schedule_tx_timeout(void);
inline void glossy_stop_tx_timeout(void);
inline void glossy_schedule_overhearing_timeout(uint8_t error_type);
inline void glossy_stop_overhearing_timeout();
inline uint8_t bit_count(uint8_t byte);
inline void initiator_notify_oap();
inline void start_oap();
inline void changeChannels(uint8_t curr_oap_round_num);
inline void start_oap_timer();
inline void stop_oap_timer(void);
inline void start_lrt(uint8_t firstime_lrt_starting);
inline void stop_lrt();
inline void update_flash(unsigned long bv_block_local, unsigned long bv_bit_pos_local);

/** @} */

/* ----------------------- Interrupt functions ---------------------- */
/**
 * \defgroup glossy_interrupts Interrupt functions
 * @{
 */

inline void glossy_begin_rx(void);
inline void glossy_end_rx(uint8_t tx_on_radio);
inline void glossy_begin_tx(void);
inline void glossy_end_tx(void);

/** @} */

/**
 * \defgroup glossy_capture Timer capture of clock ticks
 * @{
 */

/* -------------------------- Clock Capture ------------------------- */
/**
 * \brief Capture next low-frequency clock tick and DCO clock value at that instant.
 * \param t_cap_h variable for storing value of DCO clock value
 * \param t_cap_l variable for storing value of low-frequency clock value
 */
#define CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l) do {\
		/* Enable capture mode for timers B6 and A2 (ACLK) */\
		TBCCTL6 = CCIS0 | CM_POS | CAP | SCS; \
		TACCTL2 = CCIS0 | CM_POS | CAP | SCS; \
		/* Wait until both timers capture the next clock tick */\
		while (!((TBCCTL6 & CCIFG) && (TACCTL2 & CCIFG))); \
		/* Store the capture timer values */\
		t_cap_h = TBCCR6; \
		t_cap_l = TACCR2; \
		/* Disable capture mode */\
		TBCCTL6 = 0; \
		TACCTL2 = 0; \
} while (0)

/** @} */

/* -------------------------------- SFD ----------------------------- */

/**
 * \defgroup glossy_sfd Management of SFD interrupts
 * @{
 */

/**
 * \brief Capture instants of SFD events on timer B1
 * \param edge Edge used for capture.
 *
 */
#define SFD_CAP_INIT(edge) do {\
	P4SEL |= BV(CC2420_SFD_PIN);\
	TBCCTL1 = edge | CAP | SCS;\
} while (0)

/**
 * \brief Enable generation of interrupts due to SFD events
 */
#define ENABLE_SFD_INT()		do { TBCCTL1 |= CCIE; } while (0)

/**
 * \brief Disable generation of interrupts due to SFD events
 */
#define DISABLE_SFD_INT()		do { TBCCTL1 &= ~CCIE; } while (0)

/**
 * \brief Clear interrupt flag due to SFD events
 */
#define CLEAR_SFD_INT()			do { TBCCTL1 &= ~CCIFG; } while (0)

/**
 * \brief Check if generation of interrupts due to SFD events is enabled
 */
#define IS_ENABLED_SFD_INT()    !!(TBCCTL1 & CCIE)

/** @} */

/*
 * Glossy-PIP related
 */

#define DEF_STARTING_CHANNEL 19
#define  SPI_FLASH_INS_WREN        0x06
#define  SPI_FLASH_INS_WRDI        0x04
#define  SPI_FLASH_INS_RDSR        0x05
#define  SPI_FLASH_INS_WRSR        0x01
#define  SPI_FLASH_INS_READ        0x03
#define  SPI_FLASH_INS_FAST_READ   0x0b
#define  SPI_FLASH_INS_PP          0x02
#define  SPI_FLASH_INS_SE          0xd8
#define  SPI_FLASH_INS_BE          0xc7
#define  SPI_FLASH_INS_DP          0xb9
#define  SPI_FLASH_INS_RES         0xab

// related to parity check
#define PARITY_CHECK_FAILED 4
#define PARITY_BOUNDARY_LEN 12 // original 12
#define SPLASH_HEADER_LEN 9
#define NUM_BYTES_READ_BEFORE_MAIN_LOOP 9


#define BAD_CRC 1
#define BAD_PKT_LEN 2
#define BAD_GLOSSY_HDR 3

// related to second round OAP
#define CONTROL_MSG_LEN 6 // original 10
#define NUM_OAP_NOTIFICATIONS_LIMIT 20 // sent on pipeline
#define WAKE_ROUND_ONE_MAX_FORWARDS 5
#define FORWARDER_CHANNEL_CHANGE_TIMEOUT 40000U 
#define INITIATOR_CHANNEL_CHANGE_TIMEOUT 60000U 
#define MAX_NUM_OAP_TIMER_FIRINGS 2

// related to local recovery (perhaps can be fine tuned...)
#define LRT_UNIT 61292U //14 msecs 
#define NUM_OAP_ROUNDS 4 // including the last round of local recovery
#define LOCAL_RECOVERY_WAKEUP_ROUND 4 // number of the local recovery round
#define RIPPLE_ROUNDS_DURATION 34782609U // 8 sec for 32 KB

//#define RIPPLE_ROUNDS_DURATION 3260869U // 3/4th of a sec for 2KB
//#define RIPPLE_ROUNDS_DURATION 6521739U // for 5KB
//#define RIPPLE_ROUNDS_DURATION 11956521U // for 10KB
//#define RIPPLE_ROUNDS_DURATION 17391304U // for 15 KB
//#define RIPPLE_ROUNDS_DURATION 22826087U // for 20 KB
//#define RIPPLE_ROUNDS_DURATION 28260869U // for 25 KB
//#define RIPPLE_ROUNDS_DURATION 33695652U // for 30 KB
//#define RIPPLE_ROUNDS_DURATION 21739131U // for 11.5 KB but 23 bytes data packets....


//#define RIPPLE_ROUNDS_DURATION 43478261U // 10 sec


// timeout related
#define DATA_PKT 1
#define CTRL_PKT 2
#define WAKEUP_LOG_ADDR 64000L // 500 * 128
#define EXOR_ROUND_NUM 3


#endif /* GLOSSY_H_ */

/** @} */
