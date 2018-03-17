/**
  * nrf24.c
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

#include "nrf24.h"


/* Include BSP file */
#include "nrf24_bsp.h"

/**** Check for required BSP defines ****/
/* void NRF24_BSP_CE_SET(void)
 * Set (logic HIGH) the Chip Enable pin
 * */
#ifndef NRF24_BSP_CE_SET
#error "NRF24_BSP_CE_SET must be defined for BSP"
#endif

/* void NRF24_BSP_CE_RESET(void)
 * Reset (logic LOW) the Chip Enable pin
 */
#ifndef NRF24_BSP_CE_RESET
#error "NRF24_BSP_CE_RESET must be defined for BSP"
#endif

/* ssize_t NRF24_BSP_SPI_WRITE(uint8_t * buf, size_t len, int continue)
 * Transmit bytes on SPI. Chip Select is managed by BSP. Chip select should
 * be asserted prior to beginning transmission, and deasserted at the end.
 *
 * buf: Bytes to transmit
 * len: Length of buf
 *
 * Returns: Number of bytes transmitted, or negative for error
 */
#ifndef NRF24_BSP_SPI_WRITE
#error "NRF24_BSP_SPI_WRITE must be defined for BSP"
#endif

/* ssize_t NRF24_BSP_SPI_READ(uint8_t * buf, size_t len)
 * Read bytes over SPI. Chip Select is managed by BSP. Chip select should
 * be asserted prior to beginning reception, and deasserted at the end
 *
 * buf: Destination for received bytes
 * len: Length of buf
 *
 * Returns: Number of bytes received, or negative for error
 */
#ifndef NRF24_BSP_SPI_READ
#error "NRF24_BSP_SPI_READ must be defined for BSP"
#endif

/* These functions are the same as NRF24_BSP_SPI_READ and NRF24_BSP_SPI_WRITE,
 * except the Chip Select is not deasserted following the end of the
 * transaction.
 */
#ifndef NRF24_BSP_SPI_WRITE_CONT
#error "NRF24_BSP_SPI_WRITE_CONT must be defined for BSP"
#endif

#ifndef NRF24_BSP_SPI_READ_CONT
#error "NRF24_BSP_SPI_READ_CONT must be defined for BSP"
#endif

/* void NRF24_BSP_DEASSERT_CS(void)
 *
 * Deassert chip select.
 */
#ifndef NRF24_BSP_DEASSERT_CS
#error "NRF24_BSP_DEASSERT_CS must be defined for BSP"
#endif


/*** If assert is to be used, define the following function ****/
/* void NRF24_BSP_ASSERT_FAILED(char * message)
 * Print an assert failure message.
 *
 */
#ifdef NRF24_BSP_ASSERT_FAILED
#define ASSERT(msg, t) ((t) ? 0 : (NRF24_BSP_ASSERT_FAILED(msg), -1))
#else
#define ASSERT(msg, t) (!(t))
#endif
/**** END BSP TESTING ****/



/** Write to an NRF24 register (one byte)
 *
 * @param addr Register address to write
 * @param data byte to write into address
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = NRF24_CMD_W_REGISTER | addr;
    buf[1] = data;
    if (NRF24_BSP_SPI_WRITE(buf, 2) != 2) {
        return -1;
    } else {
        return 0;
    }
}

/** Read from an NRF24 register (one byte)
 *
 * @param addr Register address to write
 *
 * @return Read value (byte) on success, negative on failure
 */
int NRF24_ReadReg(uint8_t addr)
{
    if (NRF24_BSP_SPI_WRITE_CONT(&addr, 1) != 1) {
        NRF24_BSP_DEASSERT_CS();
        return -1;
    }

    uint8_t buf[1];
    if (NRF24_BSP_SPI_READ(buf, 1) != 1) {
        return -1;
    }

    return buf[0];
}

int NRF24_SetAddr(uint8_t * addr, size_t len, int reg)
{
    if (ASSERT("Invalid address length", ((len == NRF24_AW_3BYTE) ||
            (len == NRF24_AW_4BYTE) || (len == NRF24_AW_5BYTE)))) {
        return -1;
    }

    if (ASSERT("Invalid address",
                    ((reg == NRF24_REG_TX_ADDR) ||
                    (reg == NRF24_REG_RX_ADDR_P0) ||
                    (reg == NRF24_REG_RX_ADDR_P1)))) {
        return -1;
    }

    uint8_t buf[6];
    buf[0] = NRF24_CMD_W_REGISTER | reg;
    for (size_t i = 1; i <= len; i++) {
        buf[i] = addr[len - (i + 1)];
    }

    if (NRF24_BSP_SPI_WRITE(buf, 6) != 6) {
        return -1;
    }

    return 0;
}

/** Set a pipe address LSB
 *
 * @param pipe Pipe to set (NRF24_PIPE1 through NRF24_P5)
 * @param addr LSB of RX pipe address
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SetPipeRxAddr(int pipe, uint8_t addr)
{
    if (ASSERT("Invalid pipe", ((pipe >= NRF24_P0) && (pipe < NRF24_P5)))) {
        return -1;
    }

    pipe = pipe - NRF24_P0;
    return NRF24_WriteReg(NRF24_REG_RX_ADDR_P0 + pipe, addr);
}

/** Set the payload width of a pipe
 *
 * @param pipe Pipe to set (NRF24_P1 through NRF24_P5)
 * @param width Payload size
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SetPayloadWidth(int pipe, uint8_t width)
{
    if (ASSERT("Invalid pipe", ((pipe >= NRF24_P0) && (pipe <= NRF24_P5)))) {
        return -1;
    }

    if (ASSERT("Invalid payload width", (width <= 32))) {
        return -1;
    }

    pipe = pipe - NRF24_P0;
    return NRF24_WriteReg(NRF24_REG_RX_PW_P0 + pipe, width);
}

/** Initialize and power up (to Standby mode)
 *
 * @param init Initialization struct
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_Init(struct NRF24_InitStruct * init)
{
    /* TODO: Assert all values of init */

    NRF24_BSP_CE_RESET();

    NRF24_WriteReg(NRF24_REG_RF_CH, init->channel);
    NRF24_WriteReg(NRF24_REG_RF_SETUP, init->power | init->datarate);

    NRF24_WriteReg(NRF24_REG_SETUP_AW, init->addr_width - 2);
    NRF24_WriteReg(NRF24_REG_CONFIG, init->config);

    NRF24_SetTxAddr(init->tx_addr, init->addr_width);
    NRF24_SetRxAddr(init->rx_addr, init->addr_width);

    return 0;
}


/** Write bits in a register
 *
 * @param reg Register
 * @param mask Bit (or bits) to write
 * @param val NRF24_BIT_SET or NRF24_BIT_RESET
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_BitWrite(int reg, uint8_t mask, int val)
{
    int reg_val = NRF24_ReadReg(reg);
    if (reg < 0) {
        return -1;
    }

    reg_val = (reg_val & (~mask)) | (val & mask);
    return NRF24_WriteReg(reg, reg_val);
}


/** Set up for simplex DX
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SimplexTXSetup(void)
{
    NRF24_BSP_CE_RESET();

    NRF24_BitReset(NRF24_REG_CONFIG, NRF24_PRIM_RX);
    NRF24_BitReset(NRF24_REG_EN_AA, NRF24_ENAA_P0 | NRF24_ENAA_P1 |
            NRF24_ENAA_P2 | NRF24_ENAA_P3 | NRF24_ENAA_P4 | NRF24_ENAA_P5);

    NRF24_BitReset(NRF24_REG_SETUP_RETR,
            NRF24_ARC_0 | NRF24_ARC_1 | NRF24_ARC_2 | NRF24_ARC_3);

    NRF24_WriteReg(NRF24_REG_EN_RXADDR, 0);

    NRF24_BitSet(NRF24_REG_CONFIG, NRF24_PWR_UP);

    return 0;
}

/** Set up for simplex RX
 *
 * @return 0 on success, nonzero on failure
 */
int NRF24_SimplexRXSetup(void)
{
    NRF24_BitSet(NRF24_REG_CONFIG, NRF24_PWR_UP | NRF24_PRIM_RX);
    NRF24_BitReset(NRF24_REG_EN_AA, NRF24_ENAA_P0 | NRF24_ENAA_P1 |
            NRF24_ENAA_P2 | NRF24_ENAA_P3 | NRF24_ENAA_P4 | NRF24_ENAA_P5);

    NRF24_WriteReg(NRF24_REG_EN_RXADDR, NRF24_ERX_P0);

    NRF24_BSP_CE_SET();

    return 0;
}





