/**
  ******************************************************************************
  * File Name          : uart_stdio.c
  * Description        : Redirects standard output and input through a UART
  ******************************************************************************
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

/* This file is project-specific and must contain function definitions for BSP */
#include "uart_stdio_bsp.h"

#include <stdint.h>
#include <errno.h>
#include <unistd.h>

/**** Check for required BSP functions/macros ****/
/** ssize_t STDIO_BSP__WRITE(char * buf, size_t len)
 *
 * buf: Buffer to write
 * len: Length of buffer
 *
 * returns: Length written or negative for error, as with write(2)
 */
#ifndef STDIO_BSP_WRITE
#error "STDIO_BSP_WRITE must be defined for uart_stdio BSP"
#endif

/** ssize_t STDIO_BSP_WRITE_ERR(char * buf, size_t len)
 *
 * buf: Buffer to write
 * len: Length of buffer
 *
 * returns: Length written or negative for error, as with write(2)
 */
#ifndef STDIO_BSP_WRITE_ERR
#error "STDIO_BSP_WRITE_ERR must be defined for uart_stdio BSP"
#endif

/** ssize_t STDIO_BSP_READ(char * buf, size_t len)
 *
 * buf: Read destination buffer
 * len: Length of buffer
 *
 * returns: Length read or negative for error, as with read(2)
 */
#ifndef STDIO_BSP_READ
#error "STDIO_BSP_READ must be defined for uart_stdio BSP"
#endif

/**** END BSP Checks ****/


#undef errno
extern int errno;

int _read(int fd, char * buf, size_t len)
{
	if (fd == STDIN_FILENO) {
		len = STDIO_BSP_READ(buf, len);
		return len;
	} else {
		errno = EBADF;
		return -1;
	}
}

int _write(int fd, char * buf, size_t len)
{
	if (fd == STDOUT_FILENO) {
		len = STDIO_BSP_WRITE(buf, len);

		return len;
	} else if (fd == STDERR_FILENO) {
		len = STDIO_BSP_WRITE_ERR(buf, len);

		return len;
	}  else {
		errno = EBADF;
		return -1;
	}
}














