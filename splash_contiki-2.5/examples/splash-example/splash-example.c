/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 * $Id: example-collect.c,v 1.16 2011/01/10 15:11:44 adamdunkels Exp $
 */

/**
 * \file
 *         Example of how the collect primitive works.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "sys/ctimer.h"

#include "net/netstack.h"
#include "splash-example.h"
#include "dev/splash_plus_glossy.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include <stdlib.h>
#define DEGREE 20
#define RIPPLE_AUTOACK (1 << 4)
#define RIPPLE_ADR_DECODE (1 << 11)	


static void ripple_setreg(enum cc2420_register regname, unsigned value)
{
  CC2420_WRITE_REG(regname, value);
}

static unsigned ripple_getreg(enum cc2420_register regname)
{
  unsigned reg;
  CC2420_READ_REG(regname, reg);
  return reg;
}

void ripple_flushrx(void)
{
  uint8_t dummy;

  CC2420_READ_FIFO_BYTE(dummy);
  CC2420_STROBE(CC2420_SFLUSHRX);
  CC2420_STROBE(CC2420_SFLUSHRX);
}


static struct collect_conn tc;
static glossy_data_struct glossy_data;
static unsigned long linear_equ_items[DEGREE+1];
static void get_linear_equ_items(unsigned long curr_seq_no);
static void perform_exor(uint8_t *output_pkt, uint8_t *input_pkt);

// related to nbr recovery
extern uint8_t bv[(NUM_PROGRAM_PAGES/8) + 1];
uint8_t recovering_child_bv[(NUM_PROGRAM_PAGES/8) + 1], *curr_buf_for_exor, *recovery_packet, *recvd_program_packet;
void start_nbr_recovery(uint8_t open_conn);
static uint16_t rel_num_pkts=0, recovering_child_bv_next_index=0;
rimeaddr_t recovering_child;
static void adjust_bit_vector_based_on_lc();
static void u_send_data(unsigned long seq_no);
static uint8_t prev_nbr_list_index=0;
struct collect_neighbor *syn_nbr;
uint8_t mac_acks=0;
uint16_t num_recovering_child_pkts=0;
uint8_t good_link_etx_th=GOOD_LINK_ETX_THRESHOLD;


/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  /*printf("Sink got message from %d.%d, seqno %d, hops %d: len %d '%s'\n",
	 originator->u8[0], originator->u8[1],
	 seqno, hops,
	 packetbuf_datalen(),
	 (char *)packetbuf_dataptr());*/
}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
static uint8_t mode=COLLECTION, nbr_friend=0;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_collect_process, ev, data)
{
  	
  //unsigned long  bv_block, bv_bit_pos;
  static struct etimer periodic;
  static struct etimer et;
  static struct etimer print_stats_timer;
  static struct etimer start_reprogramming_timer, initiator_stop_collection_timer, stop_collection_timer;
  const rimeaddr_t *parent;
  uint8_t *serial_input, command, program_packet_len, *program_packet, *exor_packet, num_bytes_for_parity_calc=0;
  uint16_t i=0, j=0, k=0, curr_pkt_num_ones=0;
  unsigned short tmp_rand;
  unsigned long next_page=0, num_pages=0, oap_second_round_page=0,  oap_third_round_page=0;
  glossy_data_struct *glossy_data_ptr;
  uint16_t reg;
  
  PROCESS_BEGIN();

  /*
   * Build dissemination tree at a lower power. This 
   * lets us to minimize the quality differences that 
   * exists among different channels.
   */
  cc2420_set_txpower(11);
	
  srand(node_id);
  collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);
  if(node_id == INITIATOR_NODE_ID) {
	collect_set_sink(&tc, 1);
  }

  /* Allow some time for the network to settle. */
  etimer_set(&et, 300 * CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
 
  etimer_set(&stop_collection_timer, (CLOCK_SECOND * 600));

  while(1) {
    // serial communication (Ignore this!!!!)
    if(ev==serial_line_event_message) {
      serial_input = (char *)data;
      for(i=0; i<30; i++) {
        if(serial_input[i] == 255) {
	  if(serial_input[i-4] == 254) { // to filter undesired serial packets if any
            command = serial_input[i-3]; // see hello-world application patched with serial communication
            if(command == STOP_COLLECTION) {
              if(mode==COLLECTION) {
		etimer_stop(&print_stats_timer);
		etimer_stop(&periodic);
                mode=STOP_COLLECTION;
	      }
	      else {
		goto end;
	      }
            }
	    else {
	      goto end;
	    }
          }
          break;
        }
      }
    }
    
    // underlying link eatimation is data-driven
    if(mode==COLLECTION) {
      if(etimer_expired(&print_stats_timer)) {
        parent = &tc.actual_parent;
        printf("%u %u %u %u %u %u\n",node_id,PRINT_CTP_STATS,parent->u8[0],tc.rtmetric,tc.hopcount,collect_num_children());
        printf("%u %u %u %u %u %u %u\n",node_id,PRINT_CTP_STATS,parent->u8[0],tc.rtmetric,tc.hopcount,collect_num_children(),0);
        etimer_set(&print_stats_timer, CLOCK_SECOND * 60);
      }
      
      if(etimer_expired(&periodic)) {
        etimer_set(&periodic, (90 * CLOCK_SECOND) + (random_rand() % (CLOCK_SECOND * 60)));
        if(rimeaddr_node_addr.u8[0] != INITIATOR_NODE_ID) {
          if(etimer_expired(&et)) {
            packetbuf_clear();
            packetbuf_set_datalen(sprintf(packetbuf_dataptr(), "%s", "Hello") + 1);
            collect_send(&tc, 15);
          }
        }
      }

      if(etimer_expired(&stop_collection_timer)) {
        etimer_stop(&print_stats_timer);
        etimer_stop(&periodic);
        if(node_id!=INITIATOR_NODE_ID) {
          mode=STOP_COLLECTION;
        }
        else {
      	  collect_close(&tc); 
          FASTSPI_STROBE(CC2420_SRFOFF);
          etimer_set(&initiator_stop_collection_timer, 60 * CLOCK_SECOND);
          PROCESS_WAIT_UNTIL(etimer_expired(&initiator_stop_collection_timer));
          mode=STOP_COLLECTION;
        }
      }
    }
  
    // prepare for reprogramming
    if(mode==STOP_COLLECTION) {
      mode=YET_TO_START_REPROGRAMMING;
      collect_close(&tc);
      FASTSPI_STROBE(CC2420_SRFOFF);
      etimer_set(&start_reprogramming_timer, 30 * CLOCK_SECOND);
    }
   
    // start reprogramming 
    if(mode==YET_TO_START_REPROGRAMMING && etimer_expired(&start_reprogramming_timer)) {
      // update the mode
      mode=REPROGRAMMING;
        
      // erase flash (required on every node for logging otherwise only on initiator)
      next_page = EEPROMFS_ADDR_CODEPROP;
      xmem_erase(XMEM_ERASE_UNIT_SIZE, EEPROMFS_ADDR_CODEPROP);
      wait_ready();
      oap_second_round_page = NODE_ID_XMEM_OFFSET;
      xmem_erase(XMEM_ERASE_UNIT_SIZE, NODE_ID_XMEM_OFFSET);
      wait_ready();
      oap_third_round_page =  CFS_XMEM_CONF_OFFSET;
      xmem_erase(XMEM_ERASE_UNIT_SIZE,  CFS_XMEM_CONF_OFFSET);
      wait_ready();
      xmem_erase(XMEM_ERASE_UNIT_SIZE, RIPPLE_PROGRAM_OFFSET);
      wait_ready();
	
      // pre allocation as malloc may take some time...	
      program_packet_len = DATA_LEN + FOOTER_LEN + GLOSSY_HEADER_LEN;
      recvd_program_packet = (uint8_t *) malloc(program_packet_len + 1);
      curr_buf_for_exor = (uint8_t *) malloc(program_packet_len + 1);	
      recovery_packet = (uint8_t *) malloc(program_packet_len + 1);	
      
      srand(node_id);
	
      if(node_id == INITIATOR_NODE_ID) {
        // prepare program packets and then load into the flash
        program_packet_len = DATA_LEN + FOOTER_LEN + GLOSSY_HEADER_LEN;
        program_packet = (uint8_t *) malloc(program_packet_len + 1);	
        exor_packet = (uint8_t *) malloc(program_packet_len + 1);	
        program_packet[0] = program_packet_len;
 
        glossy_data.seq_no = 0;
        while(1) {
          program_packet[1] = GLOSSY_HEADER;	
          glossy_data.seq_no++;
          glossy_data.hop = 0;
          tmp_rand = (unsigned short)rand();
          tmp_rand = (unsigned short)rand();
          for(i=0; i<( DATA_LEN - ( sizeof(unsigned long) + sizeof(uint16_t)  + sizeof(uint8_t) ) ); i++) {
            tmp_rand = (unsigned short)rand();
            glossy_data.dummy_data[i] = tmp_rand % 255;
          }
          memcpy(&program_packet[2], &glossy_data, DATA_LEN);

          // prepare the packet for the parity check
          curr_pkt_num_ones = 0;
          num_bytes_for_parity_calc = (program_packet_len + 1) - (PARITY_BOUNDARY_LEN + SPLASH_HEADER_LEN);
          for(i=0; i<num_bytes_for_parity_calc; i++) {
            curr_pkt_num_ones = curr_pkt_num_ones + bit_count(program_packet[i+SPLASH_HEADER_LEN]);
          }
          glossy_data_ptr = (glossy_data_struct *)&program_packet[2];
          glossy_data_ptr->parity_num_ones = curr_pkt_num_ones;
          
          // write packet to Xmem
	  // OAP first round preparation
          xmem_pwrite((void *)program_packet, program_packet_len -1, next_page);
          wait_ready();
          next_page = next_page + 128;

          // OAP second round preparation
          program_packet[1] = GLOSSY_HEADER;
          xmem_pwrite((void *)program_packet, program_packet_len -1, oap_second_round_page);
          oap_second_round_page = oap_second_round_page + 128;
	 
          num_pages++;
          if(num_pages==NUM_PROGRAM_PAGES) {
            num_pages = 0;
            break;
          }
        }

        // Ex-OR for third round	
        for(i=1; i<=NUM_PROGRAM_PAGES; i++) {
          // init exor_packet
          memset(exor_packet, 0, program_packet_len+1);						
          // do Ex-ORing
          get_linear_equ_items(i);
          for(j=1; j<=DEGREE; j++) {
            next_page = EEPROMFS_ADDR_CODEPROP + (linear_equ_items[j]-1)*128;
            xmem_pread((void *)program_packet, program_packet_len-1, next_page);
            perform_exor(exor_packet, program_packet); // exor_packet = exor_packet + program_packet
          }

          // fill the header
          exor_packet[0] = program_packet_len;
          exor_packet[1] = GLOSSY_HEADER;
          glossy_data_ptr = (glossy_data_struct *)&exor_packet[2];
          glossy_data_ptr->seq_no = i;
          glossy_data_ptr->hop = 0;
          curr_pkt_num_ones = 0;
          num_bytes_for_parity_calc = (program_packet_len + 1) - (PARITY_BOUNDARY_LEN + SPLASH_HEADER_LEN);
          for(k=0; k<num_bytes_for_parity_calc; k++) {
            curr_pkt_num_ones = curr_pkt_num_ones + bit_count(exor_packet[k+SPLASH_HEADER_LEN]);
          }
          glossy_data_ptr->parity_num_ones = curr_pkt_num_ones;

          // write back into the flash
          xmem_pwrite((void *)exor_packet, program_packet_len -1, oap_third_round_page);
          oap_third_round_page = oap_third_round_page + 128;
        }
        free(exor_packet);
        free(program_packet);
      }
  
      // start reprogramming
      process_start(&glossy_process, NULL);
      
      /*
       * Dissemination is carried out using the maximum 
       * transmission power of 0 dBm (31) on a tree that is 
       * built using a lower power (-10 dBm). This lets us 
       * to minimize the quality differences that exists among 
       * different channels.
       */
      cc2420_set_txpower(31);
      
      parent = &tc.actual_parent;
      printf("%u %u %u %u %u %u\n",node_id,PRINT_CTP_STATS,parent->u8[0],tc.rtmetric,tc.hopcount, collect_num_children());
      printf("%u %u %u %u %u %u %u\n",node_id,PRINT_CTP_STATS,parent->u8[0],tc.rtmetric,tc.hopcount,collect_num_children(),0);
      if(node_id == INITIATOR_NODE_ID) {
        printf("start:\n");
        printf("sai start:\n");
        splash_start((uint8_t *)&glossy_data, DATA_LEN, GLOSSY_INITIATOR, GLOSSY_NO_SYNC, N_TX, tc.hopcount, collect_num_children());
      }
      else {
        printf("start:\n");
        printf("sai start:\n");
        splash_start((uint8_t *)&glossy_data, DATA_LEN, GLOSSY_RECEIVER, GLOSSY_NO_SYNC, N_TX, tc.hopcount, collect_num_children());
      }
    }
	
    if(mode==REPROGRAMMING && ev == PROCESS_EVENT_POLL) {
      mode = REPROGRAMMING_DONE;
      /* 
       * Reduce power for local recovery to increase spatial reuse.
       */ 
      cc2420_set_txpower(23);
      if(node_id != INITIATOR_NODE_ID) {
        reg = ripple_getreg(CC2420_MDMCTRL0);
        reg |= RIPPLE_AUTOACK | RIPPLE_ADR_DECODE;
        ripple_setreg(CC2420_MDMCTRL0, reg);
        ripple_flushrx();
        mac_acks = 1;
        start_nbr_recovery(1); // although full, a node has to do this as it may have to serve another node
      }
    }
end:
    PROCESS_WAIT_EVENT();
  } // end while
  PROCESS_END();
}


static void perform_exor(uint8_t *output_pkt, uint8_t *input_pkt)
{
        glossy_data_struct *glossy_data_ptr_input_pkt, *glossy_data_ptr_output_pkt;
        uint16_t i;

        glossy_data_ptr_output_pkt = (glossy_data_struct *)&output_pkt[2];
        glossy_data_ptr_input_pkt = (glossy_data_struct *)&input_pkt[2];
        for(i=0; i<( DATA_LEN - ( sizeof(unsigned long) + sizeof(uint16_t) + sizeof(uint8_t)) ); i++) {
                glossy_data_ptr_output_pkt->dummy_data[i] = glossy_data_ptr_output_pkt->dummy_data[i] ^ glossy_data_ptr_input_pkt->dummy_data[i];
        }
}

static void get_linear_equ_items(unsigned long curr_seq_no)
{
        unsigned short tmp_rand, duplicate=0, i, num_items=0;

        srand(curr_seq_no);
        linear_equ_items[1] = curr_seq_no;
        num_items++;
        while(1) {
                tmp_rand = (unsigned short)rand();
                duplicate=0;
                for(i=1; i<=num_items; i++) {
                        if(linear_equ_items[i] == tmp_rand) {
                                duplicate=1;
                        }
                }
                if(!duplicate) {
                        num_items++;
                        linear_equ_items[num_items] = (tmp_rand % (NUM_PROGRAM_PAGES)) + 1;
                        if(num_items==DEGREE) {
                                break;
                        }
                }
        }
}

// broadcast syn
void b_syn_recv(struct broadcast_conn *c, const rimeaddr_t *from);
//static struct broadcast_conn b_syn_conn;
static const struct broadcast_callbacks b_syn_callback = {b_syn_recv};
static uint8_t recovery_state=RECOVERY_STATE_IDLE;

// unicast syn
void u_syn_recv(struct unicast_conn *c, const rimeaddr_t *from);
void u_syn_sent(struct unicast_conn *c, int status, int transmissions);
static const struct unicast_callbacks u_syn_callback = {u_syn_recv, u_syn_sent};
static struct unicast_conn u_syn_conn;

// unicast synack
void u_synack_recv(struct unicast_conn *c, const rimeaddr_t *from);
void u_synack_sent(struct unicast_conn *c, int status, int transmissions);
static const struct unicast_callbacks u_synack_callback = {u_synack_recv, u_synack_sent};
static struct unicast_conn u_synack_conn;

// unicat thirdack
void u_thirdack_recv(struct unicast_conn *c, const rimeaddr_t *from);
void u_thirdack_sent(struct unicast_conn *c, int status, int transmissions);
static const struct unicast_callbacks u_thirdack_callback = {u_thirdack_recv, u_thirdack_sent};
static struct unicast_conn u_thirdack_conn;

// unicat data ack
void u_datapkt_recv(struct unicast_conn *c, const rimeaddr_t *from);
void u_datapkt_sent(struct unicast_conn *c, int status, int transmissions);
static const struct unicast_callbacks u_datapkt_callback = {u_datapkt_recv, u_datapkt_sent};
static struct unicast_conn u_datapkt_conn;


// timeout timers
static struct ctimer synack_timeout_timer;
static struct ctimer syn_tx_timer;
static struct ctimer thirdack_timeout_timer;
static struct ctimer datapkt_timeout_timer;
void start_synack_timeout_timer();
void start_thirdack_timeout_timer();
void start_datapkt_timeout_timer();
void synack_timeout_callback(void *ptr);
void thirdack_timeout_callback(void *ptr);
void syn_tx_callback(void *ptr);
void datapkt_timeout_callback(void *ptr);

// receive callbacks
void b_syn_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
	synack_t synack_pkt;

	synack_pkt.src_addr = node_id;
	if(recovery_state==RECOVERY_STATE_IDLE && rel_num_pkts==NUM_PROGRAM_PAGES) {
		recovery_state = RECOVERY_STATE_THIRDACK_WAITING;
		rimeaddr_copy(&recovering_child, from);
		// copy the bv
		memcpy(recovering_child_bv, packetbuf_dataptr(), ((NUM_PROGRAM_PAGES/8)+1)); 
		packetbuf_copyfrom(&synack_pkt, sizeof(synack_t));
		unicast_send(&u_synack_conn, from);
		start_thirdack_timeout_timer();
	}	
}

void u_syn_recv(struct unicast_conn *c, const rimeaddr_t *from)
{
	synack_t synack_pkt;
	uint16_t i, num_recvd_pkts_by_child=0; 
	unsigned long bv_block, bv_bit_pos;
	

	synack_pkt.src_addr = node_id;
	if(recovery_state==RECOVERY_STATE_IDLE && rel_num_pkts==NUM_PROGRAM_PAGES && node_id != INITIATOR_NODE_ID) {
		recovery_state = RECOVERY_STATE_THIRDACK_WAITING;
		num_recovering_child_pkts = 0;
		rimeaddr_copy(&recovering_child, from);
		// copy the bv
		memcpy(recovering_child_bv, packetbuf_dataptr(), ((NUM_PROGRAM_PAGES/8)+1)); 
               	for(i=1; i<=NUM_PROGRAM_PAGES; i++) {
                        bv_block = i/8;
                        bv_bit_pos = i % 8;
                        if( (recovering_child_bv[bv_block] & (1<<bv_bit_pos)) ) {
				num_recvd_pkts_by_child++;
                        }
                }

		printf("syn recv: %u %u\n", from->u8[0], num_recvd_pkts_by_child);
		num_recovering_child_pkts = NUM_PROGRAM_PAGES - num_recvd_pkts_by_child;
		
		packetbuf_clear(); 
		packetbuf_copyfrom(&synack_pkt, sizeof(synack_t));
		unicast_send(&u_synack_conn, from);
	  	//printf("synack sent: %u\n", from->u8[0]);
		//start_thirdack_timeout_timer();
	}	
}

void u_synack_recv(struct unicast_conn *c, const rimeaddr_t *from)
{
	thirdack_t thirdack_pkt;
	
	thirdack_pkt.src_addr = node_id;
	if(recovery_state == RECOVERY_STATE_SYNACK_WAITING) {
		recovery_state = RECOVERY_STATE_DATAPKT_WAITING;
		good_link_etx_th = GOOD_LINK_ETX_THRESHOLD;
		//printf("sai synack: %u\n", rel_num_pkts);
		// send thirdack and wait for program packets
		printf("synack recv: %u\n", from->u8[0]);
		nbr_friend = from->u8[0];
		packetbuf_clear();
		packetbuf_copyfrom(&thirdack_pkt, sizeof(thirdack_t));	
		unicast_close(&u_thirdack_conn);
		unicast_open(&u_thirdack_conn, 147, &u_thirdack_callback);
		unicast_send(&u_thirdack_conn, from);
		//printf("thirdack sent: %u\n", from->u8[0]);
		//start_datapkt_timeout_timer();
	}
	return;
}

void u_thirdack_recv(struct unicast_conn *c, const rimeaddr_t *from) 
{
	uint16_t i;
	unsigned long  bv_block, bv_bit_pos;
	
	if(recovery_state==RECOVERY_STATE_THIRDACK_WAITING) {
		recovery_state = RECOVERY_STATE_DATA_TX;
		printf("thirdack recv: %u\n", recovering_child.u8[0]);
		//printf("sai thirdack: %u\n", recovering_child.u8[0]);
		for(i=1; i<=NUM_PROGRAM_PAGES; i++) {
    			bv_block = i/8;
      			bv_bit_pos = i % 8;
      			if( !(recovering_child_bv[bv_block] & (1<<bv_bit_pos)) ) {
				recovering_child_bv_next_index = i + 1;
				u_send_data(i);
				break;
			} 				
		}
	}
	return;
}
void u_datapkt_recv(struct unicast_conn *c, const rimeaddr_t *from)
{
	unsigned long seq_no=0;
	uint16_t i, curr_rel_pkt_count=0;
  	unsigned long  bv_block, bv_bit_pos;
	uint8_t missing_packet_len=0, *missing_packet;
	unsigned long ripple_program_page=0;
	glossy_data_struct *glossy_data_ptr;

	// update state
	recovery_state = RECOVERY_STATE_DATAPKT_RECVD;

	// get seq_no
	missing_packet_len = DATA_LEN + FOOTER_LEN + GLOSSY_HEADER_LEN;
	missing_packet = (uint8_t *) malloc(missing_packet_len + 1);
	memcpy(missing_packet, packetbuf_dataptr(), missing_packet_len-1);	
	glossy_data_ptr = (glossy_data_struct *)&missing_packet[2];
	seq_no = glossy_data_ptr->seq_no;
        
	// locate bv position
	bv_block = seq_no/8;
        bv_bit_pos = seq_no % 8;
	//printf("recvd seq_no %lu from %u: \n", glossy_data_ptr->seq_no, from->u8[0]);
	
	if( !(bv[bv_block] & (1<<bv_bit_pos)) ) {
		// update bv
		bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);
		
		// write to flash
		ripple_program_page = RIPPLE_PROGRAM_OFFSET + ((seq_no - 1) * 128);
		xmem_pwrite((void *)missing_packet, missing_packet_len -1, ripple_program_page);
	
		for(i=1; i<=NUM_PROGRAM_PAGES; i++) {
			bv_block = i/8;
                	bv_bit_pos = i % 8;
			if( bv[bv_block] & (1<<bv_bit_pos) ) {
				curr_rel_pkt_count++;
			}
		}
		//printf("sai num recovered: %u\n", curr_rel_pkt_count);
		if(curr_rel_pkt_count == NUM_PROGRAM_PAGES) {
			recovery_state = RECOVERY_STATE_IDLE; // go back so that I too can serve others now
			rel_num_pkts = NUM_PROGRAM_PAGES;
			printf("end:\n");
			printf("sai end:\n");
		}
	}
	free(missing_packet);
}

// sent callbacks
void u_syn_sent(struct unicast_conn *c, int status, int transmissions)
{
	printf("syn sent: %u num_tx: %d status: %d\n", nbr_friend, transmissions, status);	
	start_synack_timeout_timer();
}

void u_synack_sent(struct unicast_conn *c, int status, int transmissions)
{
	start_thirdack_timeout_timer();
}

void u_thirdack_sent(struct unicast_conn *c, int status, int transmissions)
{
	printf("thirdack sent: %u num_tx: %d status: %d\n", nbr_friend, transmissions, status);	
	start_datapkt_timeout_timer();
}

void u_datapkt_sent(struct unicast_conn *c, int status, int transmissions)
{
	
	uint16_t i;
	unsigned long  bv_block, bv_bit_pos;

	printf("num_tx: %d status: %d\n", transmissions, status);

	// go back to IDLE state so that you can serve others..
	num_recovering_child_pkts--;
	if(num_recovering_child_pkts == 0) {
		recovery_state = RECOVERY_STATE_IDLE;		
	}

	// transmit next packet...
	for(i=recovering_child_bv_next_index; i<=NUM_PROGRAM_PAGES; i++) {
		bv_block = i/8;
		bv_bit_pos = i % 8;
		if( !(recovering_child_bv[bv_block] & (1<<bv_bit_pos)) ) {
			recovering_child_bv_next_index = i + 1;
			u_send_data(i);
			break;
		}
	}
}

static void u_send_data(unsigned long seq_no)
{
	unsigned long ripple_program_page=0;
	uint8_t missing_packet_len=0, *missing_packet;
	//glossy_data_struct *glossy_data_ptr;
	
	// malloc		
	missing_packet_len = DATA_LEN + FOOTER_LEN + GLOSSY_HEADER_LEN;
	missing_packet = (uint8_t *) malloc(missing_packet_len + 1);
	
	// read from ripple's program flash area
	ripple_program_page = RIPPLE_PROGRAM_OFFSET + ((seq_no - 1) * 128);
	xmem_pread((void *)missing_packet, missing_packet_len-1, ripple_program_page);
	
	// send it on the radio
	packetbuf_clear();
	packetbuf_copyfrom(missing_packet, missing_packet_len-1);	
	packetbuf_set_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS, RIPPLE_DATA_PKT_MAX_RTXS);
	free(missing_packet);
	
	/*missing_packet = (uint8_t *) packetbuf_dataptr();
	glossy_data_ptr = (glossy_data_struct *)&missing_packet[2];
	seq_no = glossy_data_ptr->seq_no;
	printf("seq_no sent: %lu\n", seq_no);*/
	
	unicast_send(&u_datapkt_conn, &recovering_child);
}

void start_nbr_recovery(uint8_t open_conn)
{
	uint16_t i;
	unsigned long  bv_block, bv_bit_pos;
	struct collect_neighbor *n;
	uint8_t nbr_list_index;
	uint16_t tmp_rel_num_pkts=0;
	

	nbr_list_index = 0;	
	// open required connections
	if(open_conn) {
		// adjust bit vector based on the linear coding

		tmp_rel_num_pkts=0;
		for(i=1; i<=NUM_PROGRAM_PAGES; i++) {
			bv_block = i/8;
      			bv_bit_pos = i % 8;
      			if( bv[bv_block] & (1<<bv_bit_pos) ) {
				tmp_rel_num_pkts++;
			}
		}
                printf("local recovery start time %u\n", tmp_rel_num_pkts);
                printf("sai local recovery start time %u\n", tmp_rel_num_pkts);


		adjust_bit_vector_based_on_lc(); 
		rel_num_pkts=0;
		for(i=1; i<=NUM_PROGRAM_PAGES; i++) {
			bv_block = i/8;
      			bv_bit_pos = i % 8;
      			if( bv[bv_block] & (1<<bv_bit_pos) ) {
				rel_num_pkts++;
			}
		}
                printf("nbr recovery mode: %u\n", rel_num_pkts);
                printf("sai nbr recovery mode: %u\n", rel_num_pkts);
		//broadcast_open(&b_syn_conn, 129, &b_syn_callback);
		unicast_open(&u_syn_conn, 149, &u_syn_callback);		
		unicast_open(&u_synack_conn, 146, &u_synack_callback);		
		unicast_open(&u_thirdack_conn, 147, &u_thirdack_callback);		
		unicast_open(&u_datapkt_conn, 148, &u_datapkt_callback);		
	}


back:	if(rel_num_pkts != NUM_PROGRAM_PAGES) {
		syn_nbr = NULL;	
		for(n = list_head(tc.neighbor_list.list); n != NULL; n = list_item_next(n)) {
			nbr_list_index++;
			if(nbr_list_index > prev_nbr_list_index && (n->le.etx_accumulator<=good_link_etx_th)) {
				prev_nbr_list_index = nbr_list_index;
				syn_nbr = n;
				if(open_conn) {
	 				ctimer_set(&syn_tx_timer, INIT_TIMEOUT_FOR_SYN_TX, syn_tx_callback, NULL);
				}
				else {
					syn_tx_callback(NULL);
				}
				break;
			}
		}	
		if(n==NULL) {
			prev_nbr_list_index=0;
			// time to reiterate
			if(syn_nbr == NULL) {
				nbr_list_index = 0;
				good_link_etx_th = IQ_LINK_ETX_THRESHOLD;
				goto back;		
			}
		}
	}

	return;
}

void syn_tx_callback(void *ptr)
{
	packetbuf_clear();
	packetbuf_copyfrom(bv, ((NUM_PROGRAM_PAGES/8)+1)); //broadcast_send(&b_syn_conn);
	unicast_close(&u_syn_conn);
	unicast_open(&u_syn_conn, 149, &u_syn_callback);
	unicast_send(&u_syn_conn, &(syn_nbr->addr));
	recovery_state = RECOVERY_STATE_SYNACK_WAITING;
	//start_synack_timeout_timer();
	nbr_friend = syn_nbr->addr.u8[0];
	//printf("syn sent: %u\n", syn_nbr->addr.u8[0]); 
}

void synack_timeout_callback(void *ptr)
{
  if(recovery_state == RECOVERY_STATE_SYNACK_WAITING) {
		recovery_state = RECOVERY_STATE_IDLE;
		start_nbr_recovery(0);
  }
}
void thirdack_timeout_callback(void *ptr)
{
	if(recovery_state == RECOVERY_STATE_THIRDACK_WAITING) {
		recovery_state = RECOVERY_STATE_IDLE;
	}
}

void datapkt_timeout_callback(void *ptr)
{
	if(recovery_state == RECOVERY_STATE_DATAPKT_WAITING) {
		recovery_state = RECOVERY_STATE_IDLE;
		start_nbr_recovery(0);
	}
}


void start_synack_timeout_timer()
{
	ctimer_set(&synack_timeout_timer, SYNACK_TIMEOUT, synack_timeout_callback, NULL);   
}
void start_thirdack_timeout_timer()
{
	 ctimer_set(&thirdack_timeout_timer, THIRDACK_TIMEOUT, thirdack_timeout_callback, NULL);
}

void start_datapkt_timeout_timer()
{
	 ctimer_set(&datapkt_timeout_timer, DATAPKT_TIMEOUT, datapkt_timeout_callback, NULL);
}

// Linear coding
static void decode(unsigned long curr_seq_no)
{
	unsigned short tmp_rand, duplicate=0, i, num_items=0;

	srand(curr_seq_no);
	linear_equ_items[1] = curr_seq_no;
	num_items++;
	while(1) {
		tmp_rand = (unsigned short)rand();
		duplicate=0;
		for(i=1; i<=num_items; i++) {
			if(linear_equ_items[i] == tmp_rand) {
				duplicate=1;
			}
		}
		if(!duplicate) {
			num_items++;
			linear_equ_items[num_items] = (tmp_rand % (NUM_PROGRAM_PAGES)) + 1;
			if(num_items==DEGREE) {
				break;
			}
		}
	}
}
#ifdef XOR_PATCH
uint8_t check_for_recovery()
#else 
void check_for_recovery()
#endif
{
	uint16_t i, j;
	uint8_t cant_recover=0;
	unsigned long  bv_block, bv_bit_pos;
	unsigned long ripple_program_page=0;
	uint8_t program_packet_len;
	glossy_data_struct *glossy_data_ptr;

	program_packet_len = DATA_LEN + FOOTER_LEN + GLOSSY_HEADER_LEN;
	memset(curr_buf_for_exor, 0, program_packet_len+1);
	for(i=1; i<=DEGREE; i++) {
		bv_block = linear_equ_items[i]/8;
		bv_bit_pos = linear_equ_items[i] % 8;	
		if( !(bv[bv_block] & (1<<bv_bit_pos)) ) {
			cant_recover=0;
			for(j=1; j<=DEGREE; j++) {
				if(i!=j) {
        				bv_block = linear_equ_items[j]/8;
         				bv_bit_pos = linear_equ_items[j] % 8;
          				if( !(bv[bv_block] & (1<<bv_bit_pos)) ) {
						cant_recover = 1;
					}
				}
			}
			if(!cant_recover) { // can recover the packet with seq_no i !!!!
				
				// first, update the bit vector			
				bv_block = linear_equ_items[i]/8;
				bv_bit_pos = linear_equ_items[i] % 8;
				bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);

				// start EXOR recovery : A + B = C where B is missing 
				// find A
				for(j=1; j<=DEGREE; j++) {
					if(i!=j) {
						ripple_program_page = RIPPLE_PROGRAM_OFFSET + ((linear_equ_items[j] - 1) * 128);
						xmem_pread((void *)recovery_packet, program_packet_len -1, ripple_program_page);
						perform_exor(curr_buf_for_exor, recovery_packet);
					}
				}
			
				// find B from A and C
				perform_exor(curr_buf_for_exor, recvd_program_packet); // recvd_program_packet = C
				
				// Write B to the RIPPLE program memory
                       		curr_buf_for_exor[0] = program_packet_len;
                        	curr_buf_for_exor[1] = GLOSSY_HEADER;
                        	glossy_data_ptr = (glossy_data_struct *)&curr_buf_for_exor[2];
                        	glossy_data_ptr->seq_no = linear_equ_items[i];
                        	glossy_data_ptr->hop = 0;
				ripple_program_page = RIPPLE_PROGRAM_OFFSET + ((linear_equ_items[i] - 1) * 128);
				xmem_pwrite((void *)curr_buf_for_exor, program_packet_len -1, ripple_program_page);
		
#ifdef XOR_PATCH
				return(1);
#else 
				return;
#endif
			}
			else {
#ifdef XOR_PATCH
				return(0);
#else 
				return;
#endif
			}
		}
	}
#ifdef XOR_PATCH
	return(0);
#endif
}	

static void adjust_bit_vector_based_on_lc()
{
	unsigned long next_page=0, oap_second_round_page=0, oap_third_round_page=0, prev_seq_no=0;
	glossy_data_struct *glossy_data_ptr;
	unsigned long ripple_program_page=0;
	uint8_t program_packet_len, recovery_check=0;
	uint16_t i=0;
 			
	// re-init the array
	for(i=0; i<=DEGREE; i++) {
		linear_equ_items[i] = 0;
	}
	
	next_page = EEPROMFS_ADDR_CODEPROP;
	oap_second_round_page = NODE_ID_XMEM_OFFSET;
	oap_third_round_page =  CFS_XMEM_CONF_OFFSET;
			
	program_packet_len = DATA_LEN + FOOTER_LEN + GLOSSY_HEADER_LEN;
			
	// count first round packets
	
	/*		
	while(1) {
		xmem_pread((void *)recvd_program_packet, program_packet_len -1, next_page);
		glossy_data_ptr = (glossy_data_struct *)&recvd_program_packet[2];
		next_page = next_page + 128;
		if(recvd_program_packet[0] == program_packet_len && recvd_program_packet[1] == GLOSSY_HEADER 
		&& glossy_data_ptr->seq_no>prev_seq_no) {
			prev_seq_no = glossy_data_ptr->seq_no;
			
			// update bit vector
			bv_block = glossy_data_ptr->seq_no/8;
     			bv_bit_pos = glossy_data_ptr->seq_no % 8;
      			bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);
			
			// write to program page
			ripple_program_page = RIPPLE_PROGRAM_OFFSET + ((glossy_data_ptr->seq_no - 1) * 128);
			xmem_pwrite((void *)recvd_program_packet, program_packet_len -1, ripple_program_page);
			
		}
		else { 	
    			break;
		}
	} // end while

	// count second round packets
	prev_seq_no = 0;
  
	while(1) {
  		xmem_pread((void *)recvd_program_packet, program_packet_len -1, oap_second_round_page);
		glossy_data_ptr = (glossy_data_struct *)&recvd_program_packet[2];
		oap_second_round_page = oap_second_round_page + 128;
    		if(recvd_program_packet[0] == program_packet_len && recvd_program_packet[1] == GLOSSY_HEADER 
		&& glossy_data_ptr->seq_no>prev_seq_no) {
    			prev_seq_no = glossy_data_ptr->seq_no;
			
      			bv_block = glossy_data_ptr->seq_no/8;
      			bv_bit_pos = glossy_data_ptr->seq_no % 8;
			if( !(bv[bv_block] & (1<<bv_bit_pos)) ) {
				// update bit vector
      				bv[bv_block] = bv[bv_block] | (1<<bv_bit_pos);
				// write to program page
				ripple_program_page = RIPPLE_PROGRAM_OFFSET + ((glossy_data_ptr->seq_no - 1) * 128);
				xmem_pwrite((void *)recvd_program_packet, program_packet_len -1, ripple_program_page);
			}
		}
    		else {
   			break;
	  	}
	} // end while
	*/

	prev_seq_no = 0;
	while(1) {
		xmem_pread((void *)recvd_program_packet, program_packet_len -1, oap_third_round_page);
		glossy_data_ptr = (glossy_data_struct *)&recvd_program_packet[2];
		oap_third_round_page = oap_third_round_page + 128;
		if(recvd_program_packet[0] == program_packet_len && recvd_program_packet[1] == GLOSSY_HEADER && glossy_data_ptr->seq_no>prev_seq_no) {
    			prev_seq_no = glossy_data_ptr->seq_no;
			decode(glossy_data_ptr->seq_no); // fills linear_eqn_items
#ifdef XOR_PATCH
			recovery_check = check_for_recovery(); // from the linear_eqn_items corrsponding to the current seq_no
			if(recovery_check) {
				prev_seq_no = 0;
				oap_third_round_page =  CFS_XMEM_CONF_OFFSET;
			}
#else 
			check_for_recovery(); // from the linear_eqn_items corrsponding to the current seq_no
#endif
		}
    		else {
    			break;
		}
	} // end while
	free(recvd_program_packet);
	free(curr_buf_for_exor);
	free(recovery_packet);
}

/*---------------------------------------------------------------------------*/
