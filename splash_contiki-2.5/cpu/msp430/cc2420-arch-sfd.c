/*
 * Copyright (c) 2009, Swedish Institute of Computer Science
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
 * @(#)$Id: cc2420-arch-sfd.c,v 1.5 2010/12/16 22:49:12 adamdunkels Exp $
 */

#include "contiki.h"
#ifdef __IAR_SYSTEMS_ICC__
#include <msp430.h>
#else
#include <io.h>
#include <signal.h>
#endif

#include "dev/spi.h"
#include "dev/cc2420.h"

extern volatile uint8_t cc2420_sfd_counter;
extern volatile uint16_t cc2420_sfd_start_time;
extern volatile uint16_t cc2420_sfd_end_time;

/*---------------------------------------------------------------------------*/
/* SFD interrupt for timestamping radio packets */
/*
#ifdef __IAR_SYSTEMS_ICC__
#pragma vector=TIMERB1_VECTOR
__interrupt void
#else
interrupt(TIMERB1_VECTOR)
#endif
cc24240_timerb1_interrupt(void)
{
  int tbiv;
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  tbiv = TBIV;
  if(CC2420_SFD_IS_1) {
    cc2420_sfd_counter++;
    cc2420_sfd_start_time = TBCCR1;
  } else {
    cc2420_sfd_counter = 0;
    cc2420_sfd_end_time = TBCCR1;
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}*/
/*---------------------------------------------------------------------------*/
/*void
cc2420_arch_sfd_init(void)
{
  P4SEL = BV(CC2420_SFD_PIN);
  
  TBCTL = TBSSEL_1 | TBCLR;
  
  TBCCTL1 = CM_3 | CAP | SCS;
  TBCCTL1 |= CCIE;
  
  TBCTL |= MC1;

  TBR = RTIMER_NOW();
}*/
/*---------------------------------------------------------------------------*/
