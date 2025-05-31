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
	 @addtogroup hal_idf
	 @{
	 @file xbee_platform_idf.c
	 Platform-specific functions for use by the
	 XBee Driver on ESP-IDF target.
	 Documented in platform.h
 */

#include "xbee/platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint32_t xbee_seconds_timer()
{
	return xTaskGetTickCount() / configTICK_RATE_HZ;
}

uint32_t xbee_millisecond_timer()
{
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

int xbee_platform_init(void)
{
	return 0; //nothing to initialize here
}