/**
  * nrf24.h
  *
  *  Created on: Mar 16, 2018
  *      Author: jake
  *
  * COPYRIGHT(c) 2018 Jake Drahos
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef NRF24_H_
#define NRF24_H_

#include <stdint.h>
#include <stdlib.h>

/**** REGISTER AND BIT DEFINES ****/
/* Register addresses */
enum NRF24_REGISTER_ADDR {
	NRF24_REG_CONFIG = 0x00,
	NRF24_REG_EN_AA,
	NRF24_REG_EN_RXADDR,
	NRF24_REG_SETUP_AW,
	NRF24_REG_SETUP_RETR,
	NRF24_REG_RF_CH,
	NRF24_REG_RF_SETUP,
	NRF24_REG_STATUS,
	NRF24_REG_OBSERVE_TX,
	NRF24_REG_RPD,
	NRF24_REG_RX_ADDR_P0,
	NRF24_REG_RX_ADDR_P1,
	NRF24_REG_RX_ADDR_P2,
	NRF24_REG_RX_ADDR_P3,
	NRF24_REG_RX_ADDR_P4,
	NRF24_REG_RX_ADDR_P5,
	NRF24_REG_TX_ADDR,
	NRF24_REG_RX_PW_P0,
	NRF24_REG_RX_PW_P1,
	NRF24_REG_RX_PW_P2,
	NRF24_REG_RX_PW_P3,
	NRF24_REG_RX_PW_P4,
	NRF24_REG_RX_PW_P5,
	NRF24_REG_FIFO_STATUS,
	NRF24_REG_DYNPD = 0x1c,
	NRF24_REG_FEATURE
};

/**** Bit defines ****/
/* Config register */
#define NRF24_MASK_RX_DR    0x40
#define NRF24_MASK_TX_DS    0x20
#define NRF24_MASK_MAX_RT   0x10
#define NRF24_EN_CRC        0x08
#define NRF24_CRCO          0x04
#define NRF24_PWR_UP        0x02
#define NRF24_PRIM_RX       0x01

/* Enable Auto Acknowledgment */
#define NRF24_ENAA_P5       0x20
#define NRF24_ENAA_P4       0x10
#define NRF24_ENAA_P3       0x08
#define NRF24_ENAA_P2       0x04
#define NRF24_ENAA_P1       0x02
#define NRF24_ENAA_P0       0x01

/* Enable RX Addresses */
#define NRF24_ERX_P5        0x20
#define NRF24_ERX_P4        0x10
#define NRF24_ERX_P3        0x08
#define NRF24_ERX_P2        0x04
#define NRF24_ERX_P1        0x02
#define NRF24_ERX_P0        0x01

/* Address Widths */
#define NRF24_AW_3BYTE      0x01
#define NRF24_AW_4BYTE      0x02
#define NRF24_AW_5BYTE      0x03

/* Auto Retransmission */
#define NRF24_ARD_3         0x80
#define NRF24_ARD_2         0x40
#define NRF24_ARD_1         0x20
#define NRF24_ARD_0         0x10
#define NRF24_ARC_3         0x08
#define NRF24_ARC_2         0x04
#define NRF24_ARC_1         0x02
#define NRF24_ARC_0         0x01

/* RF Setup */
#define NRF24_CONT_WAVE     0x80
#define NRF24_RF_DR_LOW     0x20
#define NRF24_PLL_LOCK		0x10
#define NRF24_RF_DR_HIGH    0x08
#define NRF24_RF_PWR_1      0x04
#define NRF24_RF_PWR_0      0x02

/* Status */
#define NRF24_RX_DR         0x40
#define NRF24_TX_DS         0x20
#define NRF24_MAX_RT        0x10
#define NRF24_RX_P_NO_2     0x08
#define NRF24_RX_P_NO_1     0x04
#define NRF24_RX_P_NO_0     0x02
#define NRF24_TX_FULL       0x01

/* Observe TX */
#define NRF24_PLOS_CNT_3    0x80
#define NRF24_PLOS_CNT_2    0x40
#define NRF24_PLOS_CNT_1    0x20
#define NRF24_PLOS_CNT_0    0x10
#define NRF24_ARC_CNT_3     0x08
#define NRF24_ARC_CNT_2     0x04
#define NRF24_ARC_CNT_1     0x02
#define NRF24_ARC_CNT_0     0x01

/* RPD */
#define NRF24_RPD           0x01

/* FIFO Status */
#define NRF24__TX_REUSE     0x40
#define NRF24_FIFO_TX_FULL  0x20
#define NRF24_TX_EMPTY      0x10
#define NRF24_RX_FULL       0x02
#define NRF24_RX_EMPTY      0x01

/* Dynamic Payload Length */
#define NRF24_DPL_P5        0x20
#define NRF24_DPL_P4        0x10
#define NRF24_DPL_P3        0x08
#define NRF24_DPL_P2        0x04
#define NRF24_DPL_P1        0x02
#define NRF24_DPL_P0        0x01

/* Feature */
#define NRF24_EN_DPL     0x04
#define NRF24_EN_ACK_PAY 0x02
#define NRF24_EN_DYN_ACK 0x01

/**** Command defines ****/
#define NRF24_CMD_R_REGISTER    0x00
#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_W_TX_PAYLOAD  0xc0
#define NRF24_CMD_FLUSH_TX      0xe1
#define NRF24_CMD_FLUSH_RX      0xe2
#define NRF24_CMD_REUSE_TX_PL   0xe3
#define NRF24_CMD_R_RX_PL_WID   0x60

#define NRF24_CMD_W_ACK_PAYLOAD         0xc8
#define NRF24_CMD_W_TX_PAYLOAD_NO_ACK   0xb0

#define NRF24_CMD_NOP   0xff

/**** END REGISTER AND BIT DEFINES ****/

/**** Defines used for high-level communication with NRF24 driver ****/

struct NRF24_InitStruct {
    uint8_t addr_width;
    uint8_t rx_addr[5];
    uint8_t tx_addr[5];
    uint8_t channel;
    uint8_t power;
    uint8_t datarate;
    uint8_t config;
};

enum NRF24_PIPE {
    NRF24_P0,
    NRF24_P1,
    NRF24_P2,
    NRF24_P3,
    NRF24_P4,
    NRF24_P5
};

enum NRF24_IRQ {
    NRF24_IRQ_TX_DS,
    NRF24_IRQ_RX_DR,
    NRF24_IRQ_MAX_RT
};

/**** Function Definitions ****/

/** Write to an NRF24 register (one byte)
 *
 * @param addr Register address to write
 * @param data byte to write into address
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_WriteReg(uint8_t addr, uint8_t data);

/** Read from an NRF24 register (one byte)
 *
 * @param addr Register address to write
 *
 * @return Read value (byte) on success, negative on failure
 */
int NRF24_ReadReg(uint8_t addr);

/** Set the TX address
 *
 * @param addr TX address, arranged MSB at index 0 to LSB at end
 * @param len Length of the TX address
 *
 * @return 0 on success, nonzero on failure
 */
#define NRF24_SetTxAddr(addr, len) \
    NRF24_SetAddr(addr, len, NRF24_REG_TX_ADDR)

/** Set the RX address of Pipe 0
 *
 * @param addr RX address, arranged MSB at index 0 to LSB at end
 * @param len Length of the RX address
 *
 * @return 0 on success, nonzero on failure
 */
#define NRF24_SetRxAddr(addr, len) \
    NRF24_SetAddr(addr, len, NRF24_REG_RX_ADDR_P0)


/** Set the RX address of Pipe 1
 *
 * @param addr RX address, arranged MSB at index 0 to LSB at end
 * @param len Length of the RX address
 *
 * @return 0 on success, nonzero on failure
 */
#define NRF24_SetRxAddr_P1(addr, len) \
    NRF24_SetAddr(addr, len, NRF24_REG_RX_ADDR_P1)

/** Set the RX or TX address
 *
 * @param addr Address, arranged MSB at index 0 to LSB at end
 * @param len Length of the address
 * @param rxtx NRF24_REG_TX_ADDR or NRF24_REG_RX_ADDR_P0
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SetAddr(uint8_t * addr, size_t len, int);

/** Set a pipe address LSB
 *
 * @param pipe Pipe to set (NRF24_PIPE1 through NRF24_P5)
 * @param addr LSB of RX pipe address
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SetPipeRxAddr(int pipe, uint8_t addr);

/** Set the payload width of a pipe
 *
 * @param pipe Pipe to set (NRF24_P1 through NRF24_P5)
 * @param width Payload size
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SetPayloadWidth(int pipe, uint8_t width);

/** Initialize and power up (to Standby mode)
 *
 * @param init Initialization struct
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_Init(struct NRF24_InitStruct * init);

#define NRF24_BIT_SET       0xff
#define NRF24_BIT_RESET     0x00

/** Write bits in a register
 *
 * @param reg Register
 * @param mask Bit (or bits) to write
 * @param val NRF24_BIT_SET or NRF24_BIT_RESET
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_RegBitWrite(int reg, uint8_t mask, int val);

/** Set bits in a register
 *
 * @param reg Register
 * @param mask Bit (or bits) to set
 *
 * @return 0 on success, nonzero on failure
 */
#define NRF24_BitSet(reg, mask) \
    NRF24_BitWrite(reg, mask, NRF24_BIT_SET)

/** Reset bits in a register
 *
 * @param reg Register
 * @param mask Bit (or bits) to reset
 *
 * @return 0 on success, nonzero no failure
 */
#define NRF24_BitReset(reg, mask) \
    NRF24_BitWrite(reg, mask, NRF24_BIT_RESET)

/** Set up for simplex DX
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SimplexTXSetup(void);

/** Set up for simplex RX
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SimplexRXSetup(void);


#endif /* NRF24_H_ */
