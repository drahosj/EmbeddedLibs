/**
  * nrf24_bsp.h
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

#ifndef NRF24_BSP_H_
#define NRF24_BSP_H_

#include <stdint.h>

#include "main.h"
#include "spi.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_gpio.h"

#define SPI_HANDLE (&hspi1)
#define SPI_TIMEOUT 500

#define NRF24_BSP_SPI_WRITE(buf, len)       nrf24_spi_tx(buf, len, 0)
#define NRF24_BSP_SPI_WRITE_CONT(buf, len)  nrf24_spi_tx(buf, len, 1)
static inline ssize_t nrf24_spi_tx(uint8_t * buf, ssize_t len, int cont)
{
    HAL_StatusTypeDef st;
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
    st = HAL_SPI_Transmit(SPI_HANDLE, buf, len, SPI_TIMEOUT);

    ssize_t retval;
    if (st == HAL_OK) {
        retval = len;
    } else {
        retval = -1;
    }

    if (!cont) {
        HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
    }

    return retval;
}

#define NRF24_BSP_SPI_READ(buf, len)        nrf24_spi_rx(buf, len, 0)
#define NRF24_BSP_SPI_READ_CONT(buf, len)   nrf24_spi_rx(buf, len, 1)
static inline ssize_t nrf24_spi_rx(uint8_t * buf, ssize_t len, int cont)
{
    HAL_StatusTypeDef st;
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
    st = HAL_SPI_Transmit(SPI_HANDLE, buf, len, SPI_TIMEOUT);

    ssize_t retval;
    if (st == HAL_OK) {
        retval = len;
    } else {
        retval = -1;
    }

    if (!cont) {
        HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
    }

    return retval;
}

#define NRF24_BSP_CE_SET()      HAL_GPIO_WritePin(NRF_CE_GPIO_Port,         \
        NRF_CE_Pin, GPIO_PIN_SET);

#define NRF24_BSP_CE_RESET()    HAL_GPIO_WritePin(NRF_CE_GPIO_Port,         \
        NRF_CE_Pin, GPIO_PIN_RESET);

#define NRF24_BSP_DEASSERT_CS() HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,        \
        NRF_CSN_Pin, GPIO_PIN_SET);

#define NRF24_BSP_ASSERT_FAILED(msg) assert_failed((uint8_t *) msg, 0)


#endif /* NRF24_BSP_H_ */
