/**
  * uart_stdio_bsp.h
  *
  * BSP for UART_STDIO using STM32 HAL in Polled mode.
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

#ifndef UART_STDIO_BSP_H_
#define UART_STDIO_BSP_H_

#include "usart.h"
#include "stm32f0xx_hal_uart.h"

#define UART_TIMEOUT      (1000)
#define UART_HANDLE       (&huart2)
#define COLOR_ERR         ((uint8_t *) "\x1b[31m")
#define COLOR_ERR_LEN     (5)
#define COLOR_RESET       ((uint8_t *) "\x1b[0m")
#define COLOR_RESET_LEN   (4)

#define STDIO_BSP_WRITE(buf, len) STDIO_BSP_write_fn(buf, len)
static inline size_t STDIO_BSP_write_fn(char * buf, size_t len)
{
	HAL_StatusTypeDef st =  HAL_UART_Transmit(UART_HANDLE,
			(uint8_t *) buf, (uint16_t) len, UART_TIMEOUT);

	if (st == HAL_OK) {
		return len;
	} else {
		return 0;
	}
}

#define STDIO_BSP_WRITE_ERR(buf, len) STDIO_BSP_write_err_fn(buf, len)
static inline size_t STDIO_BSP_write_err_fn(char * buf, size_t len)
{
	/* Silently fail; not that important */
	HAL_UART_Transmit(UART_HANDLE, COLOR_ERR, COLOR_ERR_LEN, UART_TIMEOUT);

	HAL_StatusTypeDef st =  HAL_UART_Transmit(UART_HANDLE,
			(uint8_t *) buf, (uint16_t) len, UART_TIMEOUT);

	HAL_UART_Transmit(UART_HANDLE, COLOR_RESET, COLOR_RESET_LEN, UART_TIMEOUT);

	if (st == HAL_OK) {
		return len;
	} else {
		return 0;
	}
}

#define STDIO_BSP_READ(buf, len) STDIO_BSP_read_fn(buf, len)
static inline size_t STDIO_BSP_read_fn(char * buf, size_t len)
{
	HAL_StatusTypeDef st =  HAL_UART_Receive(UART_HANDLE,
			(uint8_t *) buf, (uint16_t) len, UART_TIMEOUT);

	if (st == HAL_OK) {
		return len;
	} else {
		return 0;
	}
}

#endif /* UART_STDIO_BSP_H_ */
