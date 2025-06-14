/*
 * Copyright (c) 2017 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */
 /**
	 @defgroup hal_efm32 HAL: EFM32/Simplicity Studio
	 @ingroup hal
	 @{
	 @file xbee_platform_efm32.c
	 Platform header for efm32
 */
#ifndef __XBEE_PLATFORM_IDF
#define __XBEE_PLATFORM_IDF

 // The necessary includes for the efm32 board
#include "driver/uart.h"
#include "soc/gpio_num.h"
#include "esp_log.h"

// stdint.h for int8_t, uint8_t, int16_t, etc. types
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <inttypes.h>
#include <endian.h>

#define _f_memcpy memcpy
#define _f_memset memset

// macros used to declare a packed structure (no alignment of elements)
// The more-flexible XBEE_PACKED() replaced PACKED_STRUCT in 2019.
#define PACKED_STRUCT		struct __attribute__ ((__packed__))
#define XBEE_PACKED(name, decl)	PACKED_STRUCT name decl

typedef uint8_t bool_t; // could instead use <stdbool.h>

typedef struct xbee_serial_t {
	uint32_t		baudrate;
	uart_port_t		port; // UART port number
	gpio_num_t		tx_pin; // TX pin number
	gpio_num_t		rx_pin; // RX pin number
	gpio_num_t		rts_pin; // RTS pin number
	gpio_num_t		cts_pin; // CTS pin number
	bool_t			hw_flow_control; // true if hardware flow control is enabled
} xbee_serial_t;

#define XBEE_MS_TIMER_RESOLUTION 1 // Our timer has 1 ms resolution
#define ZCL_TIME_EPOCH_DELTA 0

#if defined(__cplusplus)
extern "C" {
#endif
	int xbee_platform_init(void);
#define XBEE_PLATFORM_INIT() xbee_platform_init()
#if defined(__cplusplus)
}
#endif
#endif /* __XBEE_PLATFORM_EFM32 */
///@}
