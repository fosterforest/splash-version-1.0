/* -*- C -*- */
/* @(#)$Id: spi.h,v 1.7 2010/03/15 23:01:37 nifi Exp $ */

#ifndef SPI_H
#define SPI_H

/* Define macros to use for checking SPI transmission status depending
   on if it is possible to wait for TX buffer ready. This is possible
   on for example MSP430 but not on AVR. */
#ifdef SPI_WAITFORTxREADY
#define SPI_WAITFORTx_BEFORE() SPI_WAITFORTxREADY()
#define SPI_WAITFORTx_AFTER()
#define SPI_WAITFORTx_ENDED() SPI_WAITFOREOTx()
#else /* SPI_WAITFORTxREADY */
#define SPI_WAITFORTx_BEFORE()
#define SPI_WAITFORTx_AFTER() SPI_WAITFOREOTx()
#define SPI_WAITFORTx_ENDED()
#endif /* SPI_WAITFORTxREADY */

extern unsigned char spi_busy;

void spi_init(void);

/******************************************************************************************************
*  TEXAS INSTRUMENTS INC.,                                                                            *
*  MSP430 APPLICATIONS.                                                                               *
*  Copyright Texas Instruments Inc, 2004                                                              *
 *****************************************************************************************************/

/***********************************************************
	FAST SPI: Low level functions
***********************************************************/

#define FASTSPI_TX(x)\
	do {\
		SPI_WAITFORTx_BEFORE();\
		SPI_TXBUF = x;\
		SPI_WAITFORTx_AFTER();\
	} while(0)

#define FASTSPI_RX(x)\
    do {\
        SPI_TXBUF = 0;\
	    SPI_WAITFOREORx();\
		x = SPI_RXBUF;\
    } while(0)

#define FASTSPI_CLEAR_RX(x) do{ SPI_RXBUF; }while(0)

#define FASTSPI_RX_GARBAGE()\
	do {\
    	SPI_TXBUF = 0;\
		SPI_WAITFOREORx();\
		(void)SPI_RXBUF;\
	} while(0)

#define FASTSPI_TX_MANY(p,c)\
	do {\
        u8_t spiCnt;\
        for (spiCnt = 0; spiCnt < (c); spiCnt++) {\
			FASTSPI_TX(((u8_t*)(p))[spiCnt]);\
		}\
        SPI_WAITFORTx_ENDED();\
	} while(0)


#define FASTSPI_RX_WORD(x)\
	 do {\
	    SPI_TXBUF = 0;\
        SPI_WAITFOREORx();\
		x = SPI_RXBUF << 8;\
	    SPI_TXBUF = 0;\
		SPI_WAITFOREORx();\
		x |= SPI_RXBUF;\
    } while (0)

#define FASTSPI_TX_ADDR(a)\
	 do {\
		  SPI_TXBUF = a;\
		  SPI_WAITFOREOTx();\
	 } while (0)

#define FASTSPI_RX_ADDR(a)\
	 do {\
		  SPI_TXBUF = (a) | 0x40;\
		  SPI_WAITFOREOTx();\
	 } while (0)



/***********************************************************
	FAST SPI: Register access
***********************************************************/
// 	  s = command strobe
// 	  a = register address
// 	  v = register value


/*#define FASTSPI_CC_TO_TX(v) \
        do {\
                CC2420_SPI_ENABLE();\
                FASTSPI_TX_ADDR(CC2420_FSCTRL);\
                FASTSPI_TX((u8_t) ((v) >> 8));\
                FASTSPI_TX((u8_t) (v));\
                SPI_WAITFORTx_ENDED();\
                FASTSPI_TX_ADDR(CC2420_STXON);\
                CC2420_SPI_DISABLE();\
        } while (0) */

#define FASTSPI_CC_TO_TX_PHASE2(v) \
        do {\
                FASTSPI_TX((u8_t) ((v) >> 8));\
                FASTSPI_TX((u8_t) (v));\
                SPI_WAITFORTx_ENDED();\
                FASTSPI_TX_ADDR(CC2420_STXON);\
                CC2420_SPI_DISABLE();\
        } while (0)

#define FASTSPI_CC_TO_TX_PHASE1() \
        do {\
                CC2420_SPI_ENABLE();\
                FASTSPI_TX_ADDR(CC2420_FSCTRL);\
        } while (0)


// To avoid memory write

/*#define FASTSPI_TX_ASM_SHIFTED_CH_VALUE()\
        do {\
                SPI_WAITFORTx_BEFORE();\
		asm volatile("mov r5, %0" : "=r" (SPI_TXBUF) : : "r5"); \
                SPI_WAITFORTx_AFTER();\
        } while(0)

#define FASTSPI_TX_ASM_CH_VALUE()\
        do {\
                SPI_WAITFORTx_BEFORE();\
                asm volatile("mov r4, %0" : "=r" (SPI_TXBUF) : : "r4"); \
                SPI_WAITFORTx_AFTER();\
        } while(0)

#define FASTSPI_CC_TO_TX_PHASE1(v,v_shifted) \
        do {\
		asm volatile("mov %0, r4" : : "m" (v) : "r4"); \
		v_shifted = (u8_t)((v) >> 8);\
		asm volatile("mov %0, r5" : : "m" (v_shifted) : "r5"); \
                CC2420_SPI_ENABLE();\
                FASTSPI_TX_ADDR(CC2420_FSCTRL);\
        } while (0)

#define FASTSPI_CC_TO_TX_PHASE2() \
	do{\
		FASTSPI_TX_ASM_SHIFTED_CH_VALUE();\
		FASTSPI_TX_ASM_CH_VALUE();\
		SPI_WAITFORTx_ENDED();\
                FASTSPI_TX_ADDR(CC2420_STXON);\
                CC2420_SPI_DISABLE();\
	} while (0)
*/


#define FASTSPI_STROBE(s) \
    do {\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_TX_ADDR(s);\
		  CC2420_SPI_DISABLE();\
    } while (0)

#define FASTSPI_SETREG(a,v)\
	 do {\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_TX_ADDR(a);\
		  FASTSPI_TX((u8_t) ((v) >> 8));\
		  FASTSPI_TX((u8_t) (v));\
                  SPI_WAITFORTx_ENDED();\
		  CC2420_SPI_DISABLE();\
	 } while (0)


#define FASTSPI_GETREG(a,v)\
	 do {\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_RX_ADDR(a);\
		  v= (u8_t)SPI_RXBUF;\
		  FASTSPI_RX_WORD(v);\
		  clock_delay(1);\
		  CC2420_SPI_DISABLE();\
	 } while (0)

// Updates the SPI status byte

#define FASTSPI_UPD_STATUS(s)\
	 do {\
		  CC2420_SPI_ENABLE();\
		  SPI_TXBUF = CC2420_SNOP;\
		  SPI_WAITFOREOTx();\
		  s = SPI_RXBUF;\
		  CC2420_SPI_DISABLE();\
	 } while (0)

/***********************************************************
	FAST SPI: FIFO Access
***********************************************************/
// 	  p = pointer to the byte array to be read/written
// 	  c = the number of bytes to read/write
// 	  b = single data byte

#define FASTSPI_WRITE_FIFO(p,c)\
	do {\
	    CC2420_SPI_ENABLE();\
		u8_t i;\
		FASTSPI_TX_ADDR(CC2420_TXFIFO);\
		for (i = 0; i < (c); i++) {\
		    FASTSPI_TX(((u8_t*)(p))[i]);\
		}\
                SPI_WAITFORTx_ENDED();\
		CC2420_SPI_DISABLE();\
    } while (0)

#define FASTSPI_WRITE_FIFO_NOCE(p,c)\
	do {\
		FASTSPI_TX_ADDR(CC2420_TXFIFO);\
		for (u8_t spiCnt = 0; spiCnt < (c); spiCnt++) {\
		    FASTSPI_TX(((u8_t*)(p))[spiCnt]);\
		}\
                SPI_WAITFORTx_ENDED();\
    } while (0)

#define FASTSPI_READ_FIFO_BYTE(b)\
	 do {\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_RX_ADDR(CC2420_RXFIFO);\
		  (void)SPI_RXBUF;\
		  FASTSPI_RX(b);\
  		  clock_delay(1);\
		  CC2420_SPI_DISABLE();\
	 } while (0)


#define FASTSPI_READ_FIFO_NO_WAIT(p,c)\
	 do {\
		  u8_t spiCnt;\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_RX_ADDR(CC2420_RXFIFO);\
		  (void)SPI_RXBUF;\
		  for (spiCnt = 0; spiCnt < (c); spiCnt++) {\
				FASTSPI_RX(((u8_t*)(p))[spiCnt]);\
		  }\
		  clock_delay(1);\
		  CC2420_SPI_DISABLE();\
	 } while (0)



#define FASTSPI_READ_FIFO_GARBAGE(c)\
	 do {\
		  u8_t spiCnt;\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_RX_ADDR(CC2420_RXFIFO);\
		  (void)SPI_RXBUF;\
		  for (spiCnt = 0; spiCnt < (c); spiCnt++) {\
				FASTSPI_RX_GARBAGE();\
		  }\
  		  clock_delay(1);\
		  CC2420_SPI_DISABLE();\
	 } while (0)



/***********************************************************
	FAST SPI: CC2420 RAM access (big or little-endian order)
***********************************************************/
//  FAST SPI: CC2420 RAM access (big or little-endian order)
// 	  p = pointer to the variable to be written
// 	  a = the CC2420 RAM address
// 	  c = the number of bytes to write
// 	  n = counter variable which is used in for/while loops (u8_t)
//
//  Example of usage:
// 	  u8_t n;
// 	  u16_t shortAddress = 0xBEEF;
// 	  FASTSPI_WRITE_RAM_LE(&shortAddress, CC2420RAM_SHORTADDR, 2);


#define FASTSPI_WRITE_RAM_LE(p,a,c,n)\
	 do {\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_TX(0x80 | (a & 0x7F));\
		  FASTSPI_TX((a >> 1) & 0xC0);\
		  for (n = 0; n < (c); n++) {\
				FASTSPI_TX(((u8_t*)(p))[n]);\
		  }\
                  SPI_WAITFORTx_ENDED();\
		  CC2420_SPI_DISABLE();\
	 } while (0)

#define FASTSPI_WRITE_RAM_BE(p,a,c,n)                              \
  do {                                                             \
    CC2420_SPI_ENABLE();                                                  \
    FASTSPI_TX(0x80 | (a & 0x7F));                                 \
    FASTSPI_TX((a >> 1) & 0xC0);                                   \
    for (n = (c); n > 0; n--) {                                    \
      FASTSPI_TX(((uint8_t *)(p))[n - 1]);                         \
    }                                                              \
    SPI_WAITFORTx_ENDED();                                         \
    CC2420_SPI_DISABLE();                                                 \
  } while (0)

#define FASTSPI_READ_RAM_LE(p,a,c,n)\
	 do {\
		  CC2420_SPI_ENABLE();\
		  FASTSPI_TX(0x80 | (a & 0x7F));\
		  FASTSPI_TX(((a >> 1) & 0xC0) | 0x20);\
                  SPI_WAITFORTx_ENDED();\
		  SPI_RXBUF;\
		  for (n = 0; n < (c); n++) {\
				FASTSPI_RX(((u8_t*)(p))[n]);\
		  }\
		  CC2420_SPI_DISABLE();\
	 } while (0)

#endif /* SPI_H */
