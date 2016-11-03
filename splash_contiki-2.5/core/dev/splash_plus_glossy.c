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
 *         Glossy + Splash core, source file.
 * \authors
 *         Glossy: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *         Splash: Manjunath D <doddaven@comp.nus.edu.sg>
 */



#include "splash_plus_glossy.h"

//static uint8_t test_flag1=0, test_flag2=0;
static unsigned long glossy_seq_no=0;
static uint8_t initiator, sync, tx_cnt, tx_max;
static uint8_t *data, *packet, *next_packet, leaf=0;
static uint8_t data_len, packet_len;
static uint8_t bytes_read, tx_relay_cnt_last;
static volatile uint8_t state;
static rtimer_clock_t t_rx_start, t_rx_stop, t_tx_start, t_tx_stop, t_lrt_start=0, t_lrt_stop=0;
unsigned long lrt_elapsed_time=0;
static rtimer_clock_t t_rx_timeout;
//unsigned long t_rx_timeout;
static rtimer_clock_t T_irq;
static unsigned long seq_num=0;
static unsigned short ie1, ie2, p1ie, p2ie, tbiv=0;
static uint16_t rx_cnt;
static uint8_t oap_round_num=0;

static rtimer_clock_t T_slot_h, T_rx_h, T_w_rt_h, T_tx_h, T_w_tr_h, t_ref_l, T_offset_h, t_first_rx_l;
#if GLOSSY_SYNC_WINDOW
static unsigned long T_slot_h_sum;
static uint8_t win_cnt;
static unsigned long next_page=0;
static unsigned long wakeup_log_page=0;

#endif /* GLOSSY_SYNC_WINDOW */
static uint8_t relay_cnt, t_ref_l_updated, overhearing=0, num_oap_timer_firings=0;
static int8_t rssi_val=0;
static uint8_t src_addr=0, first_time=1; 
uint16_t bad_crc=0;
uint16_t bad_t_irq=0;
glossy_data_struct *pkt_parity_ptr, *global_glossy_data_ptr;
static uint8_t num_bytes_for_parity_calc=0;
static uint8_t *parity_array, num_times_oap_notified=0, forward=0, prev_forward_val=0, max_notification_forwards;
//uint8_t local_reovery_timeout_timer1_started=0, local_reovery_timeout_timer2_started=0, local_reovery_timeout_timer3_started=0;
uint8_t bv[(NUM_PROGRAM_PAGES/8) + 1];
uint8_t lrt_active=0;


// Variables for PIP-style functionality
static uint16_t receiving_channel, sending_channel, ctp_hopcount;

//__regvar __no_init uint16_t second_hop_channel @ __R5;
//register uint16_t second_hop_channel asm("r4");

/* --------------------------- Radio functions ---------------------- */
static inline void radio_flush_tx(void) {
	FASTSPI_STROBE(CC2420_SFLUSHTX);
}

static inline uint8_t radio_status(void) {
	uint8_t status;
	FASTSPI_UPD_STATUS(status);
	return status;
}

static inline void radio_on(void) {
	FASTSPI_STROBE(CC2420_SRXON);
	while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
}

static inline void radio_off(void) {
#if ENERGEST_CONF_ON
	if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
		ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	}
	if (energest_current_mode[ENERGEST_TYPE_LISTEN]) {
		ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	}
#endif /* ENERGEST_CONF_ON */
	FASTSPI_STROBE(CC2420_SRFOFF);
}

static inline void radio_flush_rx(void) {
	uint8_t dummy;
	FASTSPI_READ_FIFO_BYTE(dummy);
	FASTSPI_STROBE(CC2420_SFLUSHRX);
	FASTSPI_STROBE(CC2420_SFLUSHRX);
}

static inline void radio_abort_rx(void) {
	overhearing = 0;
	radio_flush_rx();
        if(initiator) {
          FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
        }
        else {
          FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
        }
        FASTSPI_STROBE(CC2420_SRXON);
	state = GLOSSY_STATE_ABORTED;
	if(!initiator && oap_round_num != 0) {
		start_lrt(0);
	}
}

static inline void radio_abort_tx(void) {
	radio_flush_tx();
	FASTSPI_STROBE(CC2420_SRFOFF);
        if(initiator) {
          FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
        }
        else {
          FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
        }
	FASTSPI_STROBE(CC2420_SRXON);
#if ENERGEST_CONF_ON
	if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
		ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
		ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	}
#endif /* ENERGEST_CONF_ON */
}

static inline void radio_start_tx(void) {
	FASTSPI_STROBE(CC2420_STXON);
#if ENERGEST_CONF_ON
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
}

static inline void radio_write_tx(void) {
	FASTSPI_WRITE_FIFO(packet, packet_len - 1);
}

static inline void radio_write_control_pkt(void) {
        FASTSPI_WRITE_FIFO(packet, CONTROL_MSG_LEN - 1);
}

static inline void radio_write_initiator_pkt(void) {
        FASTSPI_WRITE_FIFO(next_packet, packet_len - 1);
}

/* --------------------------- SFD interrupt ------------------------ */
interrupt(TIMERB1_VECTOR)
timerb1_interrupt(void)
{
	// compute the variable part of the delay with which the interrupt has been served
	T_irq = ((RTIMER_NOW_DCO() - TBCCR1) - 24) << 1;
	if (state == GLOSSY_STATE_RECEIVING && !CC2420_SFD_IS_1) {
		if (T_irq <= 8) {
			// NOPs (variable number) to compensate for the interrupt service delay (sec. 5.2)
			asm volatile("add %[d], r0" : : [d] "m" (T_irq)); // writing to PC (r0) and [d] is a symbolic name
			asm volatile("nop");						// irq_delay = 0
			asm volatile("nop");						// irq_delay = 2
			asm volatile("nop");						// irq_delay = 4
			asm volatile("nop");						// irq_delay = 6
			asm volatile("nop");						// irq_delay = 8
			// NOPs (fixed number) to compensate for HW variations (sec. 5.3)
			// (asynchronous MCU and radio clocks)
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
		
			FASTSPI_CC_TO_TX_PHASE2(sending_channel);
			tbiv = TBIV;
			glossy_end_rx(1); 
		} else {
			CC2420_SPI_DISABLE();
                        tbiv = TBIV;
                        prev_forward_val= forward;
                        forward = 0;
                        glossy_end_rx(0);
                        forward = prev_forward_val;
		}
	} else {
		tbiv = TBIV;
                if(tbiv == TBIV_CCR5 && !initiator) { // LRT timer fired
                        stop_lrt();
                        if(lrt_elapsed_time >= RIPPLE_ROUNDS_DURATION) {
                                state=GLOSSY_STATE_WAITING;
                                splash_stop(1);
                        }
                        else {
                                start_lrt(0);
                        }
                }
                else {
		if (state == GLOSSY_STATE_WAITING && CC2420_SFD_IS_1) { 
			if(!initiator && (oap_round_num!=0)) {
				stop_lrt();
			}
			glossy_begin_rx();
		} else {
			if (state == GLOSSY_STATE_RECEIVED && CC2420_SFD_IS_1) {
				glossy_begin_tx(); 
			} else {
				if (state == GLOSSY_STATE_TRANSMITTING && !CC2420_SFD_IS_1) {
					glossy_end_tx();
				} else {
					if (state == GLOSSY_STATE_ABORTED) {
						state = GLOSSY_STATE_WAITING;
					} else {
						if ((state == GLOSSY_STATE_WAITING) && (tbiv == TBIV_CCR4)) {
							if (rx_cnt == 0 && initiator) {
								tx_cnt = 0;
								GLOSSY_LEN_FIELD = packet_len;
								GLOSSY_HEADER_FIELD = GLOSSY_HEADER;
								if (sync) {
									GLOSSY_RELAY_CNT_FIELD = MAX_VALID_RELAY_CNT;
								}
								memcpy(&GLOSSY_DATA_FIELD, data, data_len);
								state = GLOSSY_STATE_RECEIVED;
								radio_write_tx();
								radio_start_tx();
							} else {
								overhearing = 0;
								glossy_stop_overhearing_timeout();
								FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
        							FASTSPI_STROBE(CC2420_SRXON);
							}
						} else {
							// tx timeout
							if (tbiv == TBIV_CCR5) {
								if(initiator) {
								  	glossy_stop_tx_timeout();
                							glossy_seq_no++;
								  	if(glossy_seq_no<=NUM_PROGRAM_PAGES) {
										state=GLOSSY_STATE_RECEIVED;
										FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
                								radio_start_tx();
                								radio_write_initiator_pkt();
                								xmem_pread((void *)next_packet, packet_len-1, next_page);
                								next_page = next_page + 128;
								  	}
								  	else {
										if(num_times_oap_notified < max_notification_forwards) {
											initiator_notify_oap();
										}
										else { // notified to the nw, start actual programming after a while
                                        						if(oap_round_num<NUM_OAP_ROUNDS) {
                                                						start_oap_timer();
                                        						}
											else {
												state=GLOSSY_STATE_WAITING;
												splash_stop(1);
								  			}
										}
									}	
								}
							} else {
								// rx timeout
								if (tbiv == TBIV_CCR3) {
									if (state == GLOSSY_STATE_RECEIVING) {
										CC2420_SPI_DISABLE();
										radio_abort_rx();
									}
									glossy_stop_rx_timeout();
								}
								else {
									if (tbiv == TBIV_CCR6) {
										num_oap_timer_firings++;
										if(lrt_active) {
											stop_lrt();
											start_lrt(0);
										}
										if(num_oap_timer_firings == MAX_NUM_OAP_TIMER_FIRINGS) {
											if(initiator) {
												stop_oap_timer();
						        					oap_round_num++;
        											// still there are more rounds to go...
        											if(oap_round_num<NUM_OAP_ROUNDS) {
                											num_times_oap_notified = 0;
													num_oap_timer_firings = 0;
												      max_notification_forwards = NUM_OAP_NOTIFICATIONS_LIMIT;
        											}
												changeChannels(oap_round_num);
												start_oap();
											}
											else {
												stop_oap_timer();
						        					oap_round_num++;
        											// still there are more rounds to go...
        											if(oap_round_num<NUM_OAP_ROUNDS) {
                											num_times_oap_notified = 0;
													num_oap_timer_firings = 0;
												      max_notification_forwards = NUM_OAP_NOTIFICATIONS_LIMIT;
        											}
												changeChannels(oap_round_num);
											}
										}
										else {
											stop_oap_timer();
											start_oap_timer();
										}
									}
									else {
										if (state != GLOSSY_STATE_OFF) {
										// something strange is going on: go back to the waiting state
											radio_flush_rx();
											state = GLOSSY_STATE_WAITING;
										}
									}
								}
							}
						}
					}
				}
			}
		}
		}
	}
}

/* --------------------------- Glossy process ----------------------- */
PROCESS(glossy_process, "Glossy busy-waiting process");
PROCESS_THREAD(glossy_process, ev, data) {
	PROCESS_BEGIN();

	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
		// prevent the Contiki main cycle to enter the LPM mode or
		// any other process to run while Glossy is running
		while (GLOSSY_IS_ON());
	}

	PROCESS_END();
}

static inline void glossy_disable_other_interrupts(void) {
    int s = splhigh();
	ie1 = IE1;
	ie2 = IE2;
	p1ie = P1IE;
	p2ie = P2IE;
	IE1 = 0;
	IE2 = 0;
	P1IE = 0;
	P2IE = 0;
	CACTL1 &= ~CAIE;
	DMA0CTL &= ~DMAIE;
	DMA1CTL &= ~DMAIE;
	DMA2CTL &= ~DMAIE;
	// disable etimer interrupts
	TACCTL1 &= ~CCIE;
	TBCCTL0 = 0;
	CC2420_DISABLE_FIFOP_INT();
	CC2420_CLEAR_FIFOP_INT();
	SFD_CAP_INIT(CM_BOTH);
	ENABLE_SFD_INT();
	// stop Timer B
	TBCTL = 0;
	// Timer B sourced by the DCO
	TBCTL = TBSSEL1;
	// start Timer B
	TBCTL |= MC1;
    splx(s);
    watchdog_stop();
}

static inline void glossy_enable_other_interrupts(void) {
	int s = splhigh();
	IE1 = ie1;
	IE2 = ie2;
	P1IE = p1ie;
	P2IE = p2ie;
	// enable etimer interrupts
	TACCTL1 |= CCIE;
#if COOJA
	if (TACCTL1 & CCIFG) {
		etimer_interrupt();
	}
#endif
	DISABLE_SFD_INT();
	CLEAR_SFD_INT();
	CC2420_FIFOP_INT_INIT();
	CC2420_ENABLE_FIFOP_INT();
	// stop Timer B
	TBCTL = 0;
	// Timer B sourced by the 32 kHz
	TBCTL = TBSSEL0;
	// start Timer B
	TBCTL |= MC1;
    splx(s);
    watchdog_start();
}

/* --------------------------- Main interface ----------------------- */
void splash_start(uint8_t *data_, uint8_t data_len_, uint8_t initiator_, uint8_t sync_, uint8_t tx_max_, uint16_t hopcount, uint8_t num_children) 
{
	uint16_t i;

	// init glossy variables
	data = data_;
	data_len = data_len_;
	initiator = initiator_;
	sync = sync_;
	tx_max = tx_max_;
	ctp_hopcount = hopcount;
 	src_addr = 0;
   
	// init parity array
	parity_array = (uint8_t *)malloc(256);
        for(i=0; i<=255; i++) {
		parity_array[i] = bit_count((uint8_t)i);
	}

	// init bit vector	
	for(i=0; i<((NUM_PROGRAM_PAGES/8) + 1); i++) {
		bv[i] = 0;
	}

        if(first_time) {
		next_page = EEPROMFS_ADDR_CODEPROP;
		wakeup_log_page = 64000;
		tx_cnt = 0;
		rx_cnt = 0;
		if(num_children) {
	  		leaf = 0;
		}
		else {
			leaf = 1; 
		}
	}
	forward = !leaf;
	
	// disable all interrupts that may interfere with Glossy
	glossy_disable_other_interrupts();
		
	max_notification_forwards = WAKE_ROUND_ONE_MAX_FORWARDS;        
	receiving_channel = 5 * (RF_CHANNEL - 11) + 357 + 0x4000;
        sending_channel = 5 * (RF_CHANNEL  - 11) + 357 + 0x4000;
        FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);

	// set Glossy packet length, with or without relay counter depending on the sync flag value
	packet_len = (sync) ?
			data_len + FOOTER_LEN + GLOSSY_RELAY_CNT_LEN + GLOSSY_HEADER_LEN :
			data_len + FOOTER_LEN + GLOSSY_HEADER_LEN;
	
	if(first_time) {	
		packet = (uint8_t *) malloc(packet_len + 1);
		next_packet = (uint8_t *) malloc(packet_len + 1);
		pkt_parity_ptr = (glossy_data_struct *)&packet[2];
		global_glossy_data_ptr = (glossy_data_struct *)&packet[2];
	}
	// parity check related
	// This is not same number as in example-collect.c for aome implementation issue
	num_bytes_for_parity_calc = ((packet_len + 1) - (PARITY_BOUNDARY_LEN + SPLASH_HEADER_LEN)) + NUM_BYTES_READ_BEFORE_MAIN_LOOP; 
	// set the packet length field to the appropriate value
	GLOSSY_LEN_FIELD = packet_len;
	// set the header field
	GLOSSY_HEADER_FIELD = GLOSSY_HEADER;
	if (initiator) {
		// initiator: copy the application data to the data field
		memcpy(&GLOSSY_DATA_FIELD, data, data_len);
		// set Glossy state
		state = GLOSSY_STATE_RECEIVED;
	} else {
		// receiver: set Glossy state
		state = GLOSSY_STATE_WAITING;
	}
	if (sync) {
		// set the relay_cnt field to 0
		GLOSSY_RELAY_CNT_FIELD = 0;
		// the reference time has not been updated yet
		t_ref_l_updated = 0;
	}

#if !COOJA
	// resynchronize the DCO
	msp430_sync_dco();
#endif /* COOJA */

	if (initiator) {
		radio_on();
		initiator_notify_oap();
	} else {
		radio_on();
	}
	first_time=0;
	process_poll(&glossy_process);
}

uint16_t splash_stop(uint8_t end) {
	// reenable things
	stop_lrt();
	radio_flush_rx();
	radio_flush_tx();
	glossy_enable_other_interrupts();
	FASTSPI_SETREG(CC2420_FSCTRL, (5 * (26 - 11) + 357 + 0x4000));
	FASTSPI_STROBE(CC2420_SRXON);

	state = GLOSSY_STATE_OFF; 
	process_poll(&example_collect_process);
	return rx_cnt;
}

uint16_t get_rx_cnt(void) {
	return rx_cnt;
}

uint16_t get_bad_pkts_cnt(void)
{
  return bad_crc;
}

uint8_t *get_paket()
{
  return(packet);
}

uint8_t get_src_addr(void) {
	return src_addr;
}

uint8_t get_tx_cnt(void) {
        return tx_cnt;
}


unsigned long get_seq_num(void)
{
  return seq_num;
}

int8_t get_rssi_val(void)
{
  return rssi_val;
}

uint8_t get_packet_len(void) {
  return packet_len;
}

uint8_t get_relay_cnt(void) {
	return relay_cnt;
}

rtimer_clock_t get_T_slot_h(void) {
	return T_slot_h;
}

uint8_t is_t_ref_l_updated(void) {
	return t_ref_l_updated;
}

rtimer_clock_t get_t_first_rx_l(void) {
	return t_first_rx_l;
}

rtimer_clock_t get_t_ref_l(void) {
	return t_ref_l;
}

void set_t_ref_l(rtimer_clock_t t) {
	t_ref_l = t;
}

void set_t_ref_l_updated(uint8_t updated) {
	t_ref_l_updated = updated;
}

uint8_t get_state(void) {
	return state;
}

static inline void estimate_slot_length(rtimer_clock_t t_rx_stop_tmp) {
	// estimate slot length if rx_cnt > 1
	// and we have received a packet immediately after our last transmission
	if ((rx_cnt > 1) && (GLOSSY_RELAY_CNT_FIELD == (tx_relay_cnt_last + 2))) { // +2 because you would have incremented relay counter on pkt reception
		T_w_rt_h = t_tx_start - t_rx_stop; // total time
		T_tx_h = t_tx_stop - t_tx_start; // packet Tx time at this node
		T_w_tr_h = t_rx_start - t_tx_stop; // Tx time at the sender(s)
		T_rx_h = t_rx_stop_tmp - t_rx_start; // Rx time
		rtimer_clock_t T_slot_h_tmp = (T_tx_h + T_w_tr_h + T_rx_h + T_w_rt_h) / 2;
#if GLOSSY_SYNC_WINDOW
		T_slot_h_sum += T_slot_h_tmp;
		if ((++win_cnt) == GLOSSY_SYNC_WINDOW) {
			// update the slot length estimation
			T_slot_h = T_slot_h_sum / GLOSSY_SYNC_WINDOW;
			// halve the counters
			T_slot_h_sum /= 2;
			win_cnt /= 2;
		} else {
			if (win_cnt == 1) {
				// at the beginning, use the first estimation of the slot length
				T_slot_h = T_slot_h_tmp;
			}
		}
#else
		T_slot_h = T_slot_h_tmp;
#endif /* GLOSSY_SYNC_WINDOW */
	}
}

static inline void compute_sync_reference_time(void) {
#if COOJA
	rtimer_clock_t t_cap_l = RTIMER_NOW();
	rtimer_clock_t t_cap_h = RTIMER_NOW_DCO();
#else
	// capture the next low-frequency clock tick
	rtimer_clock_t t_cap_h, t_cap_l;
	CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l);
#endif /* COOJA */
	rtimer_clock_t T_rx_to_cap_h = t_cap_h - t_rx_start;
	unsigned long T_ref_to_rx_h = (GLOSSY_RELAY_CNT_FIELD - 1) * (unsigned long)T_slot_h;
	unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long)T_rx_to_cap_h;
	rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
	// high-resolution offset of the reference time
	T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
	// low-resolution value of the reference time
	t_ref_l = t_cap_l - T_ref_to_cap_l;
	relay_cnt = GLOSSY_RELAY_CNT_FIELD - 1;
	// the reference time has been updated
	t_ref_l_updated = 1;
}

/* ----------------------- Interrupt functions ---------------------- */
inline void glossy_begin_rx(void) {
	uint8_t num_boundary_bytes;
	unsigned long  bv_block, bv_bit_pos;
	uint16_t curr_pkt_num_ones=0, i;
	int s;

	t_rx_start = TBCCR1;
	state = GLOSSY_STATE_RECEIVING;

        if(overhearing) {
		glossy_stop_overhearing_timeout();
		num_boundary_bytes = 0;
	}
	else {
		 num_boundary_bytes = 11;
	}
	// Rx timeout: packet duration + 200 us
	// (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
	t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len * 35 + 200) * 4;

	// read first byte
	while (!CC2420_FIFO_IS_1) {
		if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
			radio_abort_rx();
			return;
		}
	};
	FASTSPI_READ_FIFO_BYTE(GLOSSY_LEN_FIELD);
	bytes_read = 1;

	// read second byte
	while (!CC2420_FIFO_IS_1) {
                if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                        radio_abort_rx();
                        return;
                }
        };
        FASTSPI_READ_FIFO_BYTE(GLOSSY_HEADER_FIELD);
	bytes_read = 2;

	if (GLOSSY_LEN_FIELD != packet_len && GLOSSY_LEN_FIELD != CONTROL_MSG_LEN) {
		// overhear only data packets
		if(!overhearing && !initiator && GLOSSY_HEADER_FIELD==GLOSSY_HEADER) {
			radio_flush_rx();
			tbiv = TBIV; // clear SFD interrupt 
                	FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
               		FASTSPI_STROBE(CC2420_SRXON); // current Tx aborts
                	glossy_schedule_overhearing_timeout(BAD_PKT_LEN);
                	state = GLOSSY_STATE_WAITING; 
			if(oap_round_num != 0) {
				start_lrt(0);
			}
		}
		else {
			radio_abort_rx();
		}
		return;
	}
	
	if(GLOSSY_HEADER_FIELD == CONTROL_MSG_HEADER)   {
		// could be a corrupted
		if(GLOSSY_LEN_FIELD != CONTROL_MSG_LEN) {
			radio_abort_rx();
			return;
		}
		t_rx_timeout = t_rx_start + ((rtimer_clock_t)(CONTROL_MSG_LEN+1) * 35 + 200) * 4;
        	while (!CC2420_FIFO_IS_1) {
                	if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                        	radio_abort_rx();
                        	return;
                	}
        	};
        	FASTSPI_READ_FIFO_BYTE(packet[2]);
		bytes_read++;
		if(oap_round_num != 0) {
			if(!initiator && packet[2] != ctp_hopcount-1) {
				radio_abort_rx();
				return;
			}
			else {
				packet[2] = (uint8_t)ctp_hopcount;
			}
		}	
		glossy_schedule_rx_timeout();
		FASTSPI_CC_TO_TX_PHASE1();
		return;
	}

	if (GLOSSY_HEADER_FIELD != GLOSSY_HEADER) {
		// packet with a wrong header: abort packet reception
		if(!overhearing && !initiator && (GLOSSY_LEN_FIELD == packet_len)) { // overhear only data packets
			radio_flush_rx();
			tbiv = TBIV; // clear SFD interrupt 
                	FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
                	FASTSPI_STROBE(CC2420_SRXON); // current Tx aborts
                	glossy_schedule_overhearing_timeout(BAD_GLOSSY_HDR);
                	state = GLOSSY_STATE_WAITING; 
			if(oap_round_num != 0) {
				start_lrt(0);
			}
		}
		else {
			radio_abort_rx();
		}
		return;
	}
	
	// read third byte (parity byte)
        while (!CC2420_FIFO_IS_1) {
                if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                        radio_abort_rx();
                        return;
                }
        };
        FASTSPI_READ_FIFO_BYTE(packet[2]);
 	bytes_read = 3;

        // read fourth byte (parity byte)
        while (!CC2420_FIFO_IS_1) {
                if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                        radio_abort_rx();
                        return;
                }
        };              
        FASTSPI_READ_FIFO_BYTE(packet[3]);
        bytes_read = 4;

	for(i=1; i<=5; i++) {
		while (!CC2420_FIFO_IS_1) {
                	if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                        	radio_abort_rx();
                        	return;
                	}
        	};
		FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
		bytes_read++;
	}
	if(!initiator && ( (packet[8]<(ctp_hopcount-1)) || (packet[8]==ctp_hopcount && !overhearing) )) {
		radio_abort_rx();
		return;
	}
	else {
		packet[8] = (uint8_t)ctp_hopcount;
	}	
	
	//	bytes_read=0;
	if (packet_len > 8) {
		// split into two parts to allow parity check
		while (bytes_read < num_bytes_for_parity_calc) {
			while (!CC2420_FIFO_IS_1) {
                                if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                                        radio_abort_rx();
                                        return;
                                }
			};
			FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
			bytes_read++;
			curr_pkt_num_ones = curr_pkt_num_ones + parity_array[packet[bytes_read-1]]; // parity check
		}

                while (bytes_read <= packet_len - num_boundary_bytes) {
                        while (!CC2420_FIFO_IS_1) {
                                if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                                        radio_abort_rx();
                                        return;
                                }
                        };
                        FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
                        bytes_read++;
                }
	}
	if(!overhearing) {
		if(pkt_parity_ptr->parity_num_ones != curr_pkt_num_ones) { // need to overhear
			if(!initiator) {
				radio_flush_rx();
				tbiv = TBIV; // clear SFD interrupt 
				FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
				FASTSPI_STROBE(CC2420_SRXON); // current Tx aborts
				glossy_schedule_overhearing_timeout(PARITY_CHECK_FAILED);
				state = GLOSSY_STATE_WAITING;
				if(oap_round_num != 0) {
					start_lrt(0);
				}
				return;
			}
		}
	}
	if(overhearing) {
		overhearing = 0;
		tbiv = TBIV; // clear SFD interrupt as this packet does not need to be forwarded
		FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
        	FASTSPI_STROBE(CC2420_SRXON);
		if (GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) {

			if(oap_round_num!=EXOR_ROUND_NUM) {
				bv_block = global_glossy_data_ptr->seq_no/8;
				bv_bit_pos = global_glossy_data_ptr->seq_no % 8;
				if(!(bv[bv_block] & (1<<bv_bit_pos))) {	
					// update bit vector
					bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);
					next_page = RIPPLE_PROGRAM_OFFSET + ((global_glossy_data_ptr->seq_no - 1) * 128);
                        		// write enable 
					s = splhigh();
                        		SPI_FLASH_ENABLE();
                        		SPI_WRITE(SPI_FLASH_INS_WREN);
                        		SPI_FLASH_DISABLE();
                        		//wait_ready();
					splx(s);

                       			// program page 
					s = splhigh();
                        		SPI_FLASH_ENABLE();
                        		SPI_WRITE_FAST(SPI_FLASH_INS_PP);
                        		SPI_WRITE_FAST(next_page >> 16);
                        		SPI_WRITE_FAST(next_page >> 8);
                        		SPI_WRITE_FAST(next_page >> 0);
                        		for(i=0; i<packet_len-1; i++) {
                        		        SPI_WRITE_FAST(~packet[i]);
                        		}
                        		SPI_WAITFORTx_ENDED();
                        		SPI_FLASH_DISABLE();
					splx(s);
				}
			}
			else {
				// write enable 
                                s = splhigh();
                                SPI_FLASH_ENABLE();
                                SPI_WRITE(SPI_FLASH_INS_WREN);
                                SPI_FLASH_DISABLE();
                                //wait_ready();
                                 splx(s);

                                // program page 
                                s = splhigh();
                                SPI_FLASH_ENABLE();
                                SPI_WRITE_FAST(SPI_FLASH_INS_PP);
                                SPI_WRITE_FAST(next_page >> 16);
                                SPI_WRITE_FAST(next_page >> 8);
                                SPI_WRITE_FAST(next_page >> 0);
                                for(i=0; i<packet_len-1; i++) {
	                                SPI_WRITE_FAST(~packet[i]);
                                }
                                SPI_WAITFORTx_ENDED();
                                SPI_FLASH_DISABLE();
                                splx(s);
                                next_page = next_page + 128;
			}
		}
		start_lrt(0);
		state = GLOSSY_STATE_WAITING;
	}
	else {	
		glossy_schedule_rx_timeout();
		FASTSPI_CC_TO_TX_PHASE1();
	}
}

inline void glossy_end_rx(uint8_t tx_on_radio) {
	int i;
	unsigned long  bv_block, bv_bit_pos;
	int s;

	if(GLOSSY_HEADER_FIELD == CONTROL_MSG_HEADER) {
		glossy_stop_tx_timeout();
		FASTSPI_READ_FIFO_NO_WAIT(&packet[bytes_read], CONTROL_MSG_LEN - bytes_read + 1);
		if (packet[CONTROL_MSG_LEN] & FOOTER1_CRC_OK) { // check crc
			if(num_times_oap_notified < max_notification_forwards) {
				state = GLOSSY_STATE_RECEIVED;
				radio_write_control_pkt();
                                if(packet[3] == 4) {
                                        s = splhigh();
                                        SPI_FLASH_ENABLE();
                                        SPI_WRITE(SPI_FLASH_INS_WREN);
                                        SPI_FLASH_DISABLE();
                                        splx(s);

                                        s = splhigh();
                                        SPI_FLASH_ENABLE();
                                        SPI_WRITE_FAST(SPI_FLASH_INS_PP);
                                        SPI_WRITE_FAST(wakeup_log_page >> 16);
                                        SPI_WRITE_FAST(wakeup_log_page >> 8);
                                        SPI_WRITE_FAST(wakeup_log_page >> 0);
                                        SPI_WRITE_FAST(~1);
                                        SPI_WAITFORTx_ENDED();
                                        SPI_FLASH_DISABLE();
                                        splx(s);
                                        wakeup_log_page = wakeup_log_page + 1;
                                }
        			num_times_oap_notified++;
                		if(!initiator) {
					stop_oap_timer();
                        		if(packet[3] == 1) {
						forward = !leaf;
                                		//next_page = EEPROMFS_ADDR_CODEPROP;;
                        		}
					else {
                        			if(packet[3] == 2) {
							forward = 1;
                                			//next_page = NODE_ID_XMEM_OFFSET;
                        			}
						else {
                        				if(packet[3] == 3) {
								forward = !leaf;
                                				next_page = CFS_XMEM_CONF_OFFSET;
                        				}
						}
					}	
					start_oap_timer();
                		}
			}
			else {
				radio_abort_tx();
				if(!initiator && oap_round_num != 0) {
					start_lrt(0); // new lrt
				}
				state = GLOSSY_STATE_WAITING;
				if(oap_round_num<NUM_OAP_ROUNDS && initiator) {
					start_oap_timer();
				}
			}
		}
		else {
			radio_abort_tx();
			if(!initiator && oap_round_num != 0) {
				start_lrt(0); // new lrt
			}
			state = GLOSSY_STATE_WAITING;
			// do not wait for timeout, this may be faster than waiting for a timeout
			if(num_times_oap_notified < max_notification_forwards) {
				if(initiator && (oap_round_num!=0)) {
                                       	initiator_notify_oap();
                                }
			}
			else {
				if(oap_round_num<NUM_OAP_ROUNDS && initiator) {
                                        start_oap_timer();
                                }
			}	
		}
		return;
	}

	FASTSPI_READ_FIFO_NO_WAIT(&packet[bytes_read], packet_len - bytes_read + 1);
	if (GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) {
		if(!initiator && forward) {
			radio_write_tx();
                        if(oap_round_num!=EXOR_ROUND_NUM) {
				bv_block = global_glossy_data_ptr->seq_no/8;
				bv_bit_pos = global_glossy_data_ptr->seq_no % 8;
				if(!(bv[bv_block] & (1<<bv_bit_pos))) {	
					// update bit vector
					bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);
                                	next_page = RIPPLE_PROGRAM_OFFSET + ((global_glossy_data_ptr->seq_no - 1) * 128);
					// write enable 
					s = splhigh();
					SPI_FLASH_ENABLE();
					SPI_WRITE(SPI_FLASH_INS_WREN);
        				SPI_FLASH_DISABLE();
					//wait_ready();
					splx(s);
       
					// program page 
					s = splhigh();
        				SPI_FLASH_ENABLE();
        				SPI_WRITE_FAST(SPI_FLASH_INS_PP);
        				SPI_WRITE_FAST(next_page >> 16);
        				SPI_WRITE_FAST(next_page >> 8);
        				SPI_WRITE_FAST(next_page >> 0);
					for(i=0; i<packet_len-1; i++) {
        					SPI_WRITE_FAST(~packet[i]);
					}
       	 				SPI_WAITFORTx_ENDED();
        				SPI_FLASH_DISABLE();
					splx(s); 
				}
			}
			else {
				// write enable 
                                s = splhigh();
                                SPI_FLASH_ENABLE();
                                SPI_WRITE(SPI_FLASH_INS_WREN);
                                SPI_FLASH_DISABLE();
                                //wait_ready();
                                splx(s);

                                // program page 
                                s = splhigh();
                                SPI_FLASH_ENABLE();
                                SPI_WRITE_FAST(SPI_FLASH_INS_PP);
                                SPI_WRITE_FAST(next_page >> 16);
                                SPI_WRITE_FAST(next_page >> 8);
                                SPI_WRITE_FAST(next_page >> 0);
                                for(i=0; i<packet_len-1; i++) {
	                                SPI_WRITE_FAST(~packet[i]);
                                }
                                SPI_WAITFORTx_ENDED();
                                SPI_FLASH_DISABLE();
                                splx(s);
                                next_page = next_page + 128;
				
			}
			state = GLOSSY_STATE_RECEIVED;
		}
		else {
			if(initiator) {
				glossy_stop_tx_timeout();
				glossy_seq_no++;
				if(glossy_seq_no<=NUM_PROGRAM_PAGES) {
				
					// send next pkt, not the one just received	
					radio_write_initiator_pkt();

					// proactively read the next packet to be transmitted
					xmem_pread((void *)next_packet, packet_len-1, next_page);
					next_page = next_page + 128;
					state = GLOSSY_STATE_RECEIVED;
				}
				else {
					radio_abort_tx();
					state = GLOSSY_STATE_WAITING;
					if(num_times_oap_notified < max_notification_forwards) {
						 initiator_notify_oap();
					}
                                        else {
						state=GLOSSY_STATE_WAITING;
                                               	splash_stop(1);
					}
				}		
			}
			else {
                                radio_abort_tx();
		                if(oap_round_num!=EXOR_ROUND_NUM) {
					bv_block = global_glossy_data_ptr->seq_no/8;
					bv_bit_pos = global_glossy_data_ptr->seq_no % 8;
					if(!(bv[bv_block] & (1<<bv_bit_pos))) {	
						// update bit vector
						bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);
						next_page = RIPPLE_PROGRAM_OFFSET + ((global_glossy_data_ptr->seq_no - 1) * 128);
						// write enable 
						s = splhigh();
                                		SPI_FLASH_ENABLE();
                                		SPI_WRITE(SPI_FLASH_INS_WREN);
                                		SPI_FLASH_DISABLE();
                                		//wait_ready();
                				splx(s);

                                		// program page 
						s = splhigh();
                                		SPI_FLASH_ENABLE();
                                		SPI_WRITE_FAST(SPI_FLASH_INS_PP);
                                		SPI_WRITE_FAST(next_page >> 16);
                                		SPI_WRITE_FAST(next_page >> 8);
                                		SPI_WRITE_FAST(next_page >> 0);
                        			for(i=0; i<packet_len-1; i++) {
                                			SPI_WRITE_FAST(~packet[i]);
                        			}
                                		SPI_WAITFORTx_ENDED();
                                		SPI_FLASH_DISABLE();
						splx(s);
					}
				}
				else {
					
					// write enable 
					s = splhigh();
                                        SPI_FLASH_ENABLE();
                                        SPI_WRITE(SPI_FLASH_INS_WREN);
                                        SPI_FLASH_DISABLE();
                                        //wait_ready();
                                        splx(s);

                                        // program page 
                                        s = splhigh();
                                        SPI_FLASH_ENABLE();
                                        SPI_WRITE_FAST(SPI_FLASH_INS_PP);
                                        SPI_WRITE_FAST(next_page >> 16);
                                        SPI_WRITE_FAST(next_page >> 8);
                                        SPI_WRITE_FAST(next_page >> 0);
                                        for(i=0; i<packet_len-1; i++) {
  	                                      SPI_WRITE_FAST(~packet[i]);
                                        }
                                        SPI_WAITFORTx_ENDED();
                                        SPI_FLASH_DISABLE();
                                        splx(s);
                                        next_page = next_page + 128; 
				}
				state = GLOSSY_STATE_WAITING;
				start_lrt(0); // new lrt
			}
		}
		rx_cnt++;
	} else {
		if(initiator) {
			glossy_stop_tx_timeout();
			glossy_seq_no++;
			if(glossy_seq_no<=NUM_PROGRAM_PAGES) {
				// send next pkt, not the one just received     
				radio_write_initiator_pkt();
	                	// proactively read the next packet to be transmitted
				xmem_pread((void *)next_packet, packet_len-1, next_page);
				next_page = next_page + 128;
				state = GLOSSY_STATE_RECEIVED;
			}
			else {
				radio_abort_tx();
				state=GLOSSY_STATE_WAITING;
				if(num_times_oap_notified < max_notification_forwards) {
					initiator_notify_oap();
				}
				else {
					state=GLOSSY_STATE_WAITING;
                                       	splash_stop(1);
				}
			}
		}
		else {
			// no need to start LRT here as overhearing will take care of it

			//try to listen on the sending channel OTHERWISE you have to ABORT the TX
			FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
        		FASTSPI_STROBE(CC2420_SRXON); // current Tx aborts
			glossy_schedule_overhearing_timeout(BAD_CRC);
			state = GLOSSY_STATE_WAITING;
			start_lrt(0);		
			//radio_abort_tx();
		}
		//bad_crc++;
	}
}

inline void glossy_begin_tx(void) {
	t_tx_start = TBCCR1;
	state = GLOSSY_STATE_TRANSMITTING;
}

inline void glossy_end_tx(void) {
	// Buddy, go back to receiving channel as quickly as possible 
	//	FASTSPI_STROBE(CC2420_SRFOFF);
	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
        FASTSPI_STROBE(CC2420_SRXON);

	// anything else later
	ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	t_tx_stop = TBCCR1;
	++tx_cnt;
	state = GLOSSY_STATE_WAITING;

	if(initiator) {
		if(next_packet[1] == CONTROL_MSG_HEADER && (oap_round_num != 0)) {
			glossy_schedule_tx_timeout(CTRL_PKT);
		}
		else {
			glossy_schedule_tx_timeout(DATA_PKT);
		}
	}
	else {
		if(oap_round_num != 0) {
			start_lrt(0);
		}
	}
}

/* ------------------------------ Timeouts -------------------------- */
inline void glossy_schedule_tx_timeout(uint8_t pkt_type) {
	if(pkt_type == DATA_PKT) {
        	TBCCR5 = RTIMER_NOW_DCO() + (((rtimer_clock_t)(packet_len+1) * 35 + 200) * 4) + 4100 ; // 4000: 1ms to handle uncertainities 
	}
	else {
		if(pkt_type == CTRL_PKT) {
			TBCCR5 = RTIMER_NOW_DCO() + (((rtimer_clock_t)(CONTROL_MSG_LEN+1) * 35 + 200) * 4) + 4100;
		}
	}
        TBCCTL5 = CCIE;
}

inline void glossy_stop_tx_timeout(void) {
        TBCCTL5 = 0;
}

inline void glossy_schedule_rx_timeout(void) {
	TBCCR3 = t_rx_timeout;
	TBCCTL3 = CCIE;
}

inline void glossy_stop_rx_timeout(void) {
	TBCCTL3 = 0;
}

inline void glossy_schedule_overhearing_timeout(uint8_t error_type)
{
	//TBCCR4 = RTIMER_NOW_DCO() + 4000; // 1 ms
	if(error_type == BAD_CRC) {
		TBCCR4 = RTIMER_NOW_DCO() + (((rtimer_clock_t)(packet_len+1) * 32) * 2); // half of the packet time
	}
	if(error_type == BAD_PKT_LEN || error_type == BAD_GLOSSY_HDR) {
		TBCCR4 = RTIMER_NOW_DCO() + (((rtimer_clock_t)(packet_len+1) * 32) * 4) + (((rtimer_clock_t)packet_len * 32) * 2); // one and half packet time
	}
        if(error_type == PARITY_CHECK_FAILED) {
		TBCCR4 = RTIMER_NOW_DCO() + (((rtimer_clock_t)(packet_len+1) * 32) * 3); // 3/4 of the one pkt time
	}
	TBCCTL4 = CCIE;
	overhearing = 1;
}

inline void glossy_stop_overhearing_timeout(void) 
{
	TBCCTL4 = 0;
}

inline void glossy_schedule_initiator_timeout(void) {
	//TBCCR4 = RTIMER_NOW_DCO() + GLOSSY_INITIATOR_TIMEOUT;
	//TBCCTL4 = CCIE;
}

inline void glossy_stop_initiator_timeout(void) {
	//TBCCTL4 = 0;
}

inline void stop_oap_timer(void) 
{
	TBCCTL6 = 0;
}


inline uint8_t bit_count(uint8_t byte)
{
  uint8_t num_bits = 0;
  do {
    num_bits = num_bits + (byte & 1);
  } while (byte >>= 1);
  return num_bits;
}

inline void start_oap_timer()
{
	stop_oap_timer(); // for safety, particulalry for first round wakeup
	if(initiator) {
		if(oap_round_num!=LOCAL_RECOVERY_WAKEUP_ROUND-1) {
			TBCCR6 = RTIMER_NOW_DCO() + INITIATOR_CHANNEL_CHANGE_TIMEOUT;
			TBCCTL6 = CCIE;
		}
		else {
			state=GLOSSY_STATE_WAITING;
			splash_stop(1);
		}
	}
	else {
		TBCCR6 = RTIMER_NOW_DCO() + FORWARDER_CHANNEL_CHANGE_TIMEOUT;
		TBCCTL6 = CCIE;
	}
}

inline void start_lrt(uint8_t firstime_lrt_starting)
{
	t_lrt_start = RTIMER_NOW_DCO();
	lrt_active = 1;
	if(!firstime_lrt_starting) {
		if(t_lrt_start >  t_lrt_stop) {
			lrt_elapsed_time =  lrt_elapsed_time + (t_lrt_start - t_lrt_stop);
		}
		else {
			lrt_elapsed_time =  lrt_elapsed_time + (65535-t_lrt_stop) + t_lrt_start;
		}
	}
	TBCCR5 = RTIMER_NOW_DCO() + LRT_UNIT;
	TBCCTL5 = CCIE;
}

inline void stop_lrt()
{
	t_lrt_stop = RTIMER_NOW_DCO();
	lrt_active = 0;
	TBCCTL5 = 0;
	if(t_lrt_stop > t_lrt_start) {
		lrt_elapsed_time =  lrt_elapsed_time + (t_lrt_stop - t_lrt_start);
	}
	else {
		lrt_elapsed_time =  lrt_elapsed_time + (65535-t_lrt_start) + t_lrt_stop;
	}
}

inline void changeChannels(uint8_t curr_oap_round_num)
{
	if(curr_oap_round_num == 1) {
       		if(ctp_hopcount==0) {
                	receiving_channel = 5 * (R1_H1_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H1_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
        	}
        	if(ctp_hopcount==1) {
                	receiving_channel = 5 * (R1_H1_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H2_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
       	 	}
        	if(ctp_hopcount==2) {
                	receiving_channel = 5 * (R1_H2_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H3_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==3) {
                	receiving_channel = 5 * (R1_H3_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H4_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==4) {
                	receiving_channel = 5 * (R1_H4_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H5_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==5) {
                	receiving_channel = 5 * (R1_H5_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H6_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==6) {
                	receiving_channel = 5 * (R1_H6_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H7_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==7) {
                	receiving_channel = 5 * (R1_H7_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H8_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==8) {
                	receiving_channel = 5 * (R1_H8_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H9_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==9) {
                	receiving_channel = 5 * (R1_H9_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R1_H10_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==10) {
                	receiving_channel = 5 * (R1_H10_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (13 - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
		if(!initiator) {					
			start_lrt(1);
		}
	}

	if(curr_oap_round_num == 2) {
		FASTSPI_STROBE(CC2420_SRFOFF);
      		if(ctp_hopcount==0) {
                	receiving_channel = 5 * (R2_H1_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H1_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
        	}
        	if(ctp_hopcount==1) {
                	receiving_channel = 5 * (R2_H1_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H2_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
       	 	}
        	if(ctp_hopcount==2) {
                	receiving_channel = 5 * (R2_H2_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H3_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==3) {
                	receiving_channel = 5 * (R2_H3_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H4_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==4) {
                	receiving_channel = 5 * (R2_H4_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H5_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==5) {
                	receiving_channel = 5 * (R2_H5_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H6_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==6) {
                	receiving_channel = 5 * (R2_H6_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H7_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==7) {
                	receiving_channel = 5 * (R2_H7_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H8_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==8) {
                	receiving_channel = 5 * (R2_H8_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H9_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==9) {
                	receiving_channel = 5 * (R2_H9_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R2_H10_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==10) {
                	receiving_channel = 5 * (R2_H10_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (13 - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
	}
        if(curr_oap_round_num == 3) {
                FASTSPI_STROBE(CC2420_SRFOFF);
      		if(ctp_hopcount==0) {
                	receiving_channel = 5 * (R3_H1_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H1_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
        	}
        	if(ctp_hopcount==1) {
                	receiving_channel = 5 * (R3_H1_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H2_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
       	 	}
        	if(ctp_hopcount==2) {
                	receiving_channel = 5 * (R3_H2_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H3_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==3) {
                	receiving_channel = 5 * (R3_H3_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H4_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==4) {
                	receiving_channel = 5 * (R3_H4_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H5_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==5) {
                	receiving_channel = 5 * (R3_H5_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H6_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==6) {
                	receiving_channel = 5 * (R3_H6_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H7_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==7) {
                	receiving_channel = 5 * (R3_H7_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H8_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==8) {
                	receiving_channel = 5 * (R3_H8_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H9_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==9) {
                	receiving_channel = 5 * (R3_H9_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (R3_H10_CH - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}
        	if(ctp_hopcount==10) {
                	receiving_channel = 5 * (R3_H10_CH - 11) + 357 + 0x4000;
                	sending_channel = 5 * (13 - 11) + 357 + 0x4000;
                	FASTSPI_SETREG(CC2420_FSCTRL, receiving_channel);
			FASTSPI_STROBE(CC2420_SRXON);
        	}                
        }
	
	if(curr_oap_round_num == LOCAL_RECOVERY_WAKEUP_ROUND) {
		state=GLOSSY_STATE_WAITING;
		splash_stop(1);
	}
	
	return;
}




inline void start_oap()
{

       if(oap_round_num == 1) {

                FASTSPI_STROBE(CC2420_SRFOFF);

        	// disable
                radio_flush_tx();
                radio_flush_rx();
                DISABLE_SFD_INT();
                CLEAR_SFD_INT();

        	// enable        
                SFD_CAP_INIT(CM_BOTH);
                ENABLE_SFD_INT();

                state=GLOSSY_STATE_RECEIVED;
                glossy_seq_no = 1;
                next_page = EEPROMFS_ADDR_CODEPROP;
                xmem_pread((void *)next_packet, packet_len-1, next_page);
                next_page = next_page + 128;
                FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
                radio_start_tx();
                radio_write_initiator_pkt();
                xmem_pread((void *)next_packet, packet_len-1, next_page);
                next_page = next_page + 128;
        }

	if(oap_round_num == 2) {

        	FASTSPI_STROBE(CC2420_SRFOFF);

        	// disable
        	radio_flush_tx();
        	radio_flush_rx();
        	DISABLE_SFD_INT();
        	CLEAR_SFD_INT();

        	// enable        
        	SFD_CAP_INIT(CM_BOTH);
        	ENABLE_SFD_INT();

		state=GLOSSY_STATE_RECEIVED;
		glossy_seq_no = 1;
		next_page = NODE_ID_XMEM_OFFSET;
		xmem_pread((void *)next_packet, packet_len-1, next_page);
		next_page = next_page + 128;
		FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
		radio_start_tx();
		radio_write_initiator_pkt();
		xmem_pread((void *)next_packet, packet_len-1, next_page);
		next_page = next_page + 128;
	}
	if(oap_round_num == 3) {

        	FASTSPI_STROBE(CC2420_SRFOFF);
	
	        // disable
	        radio_flush_tx();
	        radio_flush_rx();
	        DISABLE_SFD_INT();
	        CLEAR_SFD_INT();

        	// enable        
        	SFD_CAP_INIT(CM_BOTH);
        	ENABLE_SFD_INT();


                state=GLOSSY_STATE_RECEIVED;
                glossy_seq_no = 1;
                next_page = CFS_XMEM_CONF_OFFSET;
                xmem_pread((void *)next_packet, packet_len-1, next_page);
                next_page = next_page + 128;
                FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
                radio_start_tx();
                radio_write_initiator_pkt();
                xmem_pread((void *)next_packet, packet_len-1, next_page);
                next_page = next_page + 128;
        }
}

inline void initiator_notify_oap()
{
	uint8_t curr_pkt_len;

	FASTSPI_STROBE(CC2420_SRFOFF);
	
	// disable
	radio_flush_tx();
	radio_flush_rx();
        DISABLE_SFD_INT();
        CLEAR_SFD_INT();
	
	// enable        
	SFD_CAP_INIT(CM_BOTH);
        ENABLE_SFD_INT();

	        
	state=GLOSSY_STATE_RECEIVED;
	next_packet[0] = CONTROL_MSG_LEN;
	next_packet[1] = CONTROL_MSG_HEADER;
	next_packet[2] = (uint8_t)ctp_hopcount;
	next_packet[3] = oap_round_num + 1; // current round plus 1
        FASTSPI_SETREG(CC2420_FSCTRL, sending_channel);
	radio_start_tx();
	curr_pkt_len = packet_len;
	packet_len = CONTROL_MSG_LEN;
	num_times_oap_notified++;
	radio_write_initiator_pkt();
	packet_len = curr_pkt_len; 
}

