/*
 * Copyright (c) 2019 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc., 9350 Excelsior Blvd., Suite 700, Hopkins, MN 55343
 * ===========================================================================
 */

/*
    Implementation of xbee_random() API using PSA Crypto random generation.
*/

#include <errno.h>

#include "xbee/platform.h"
#include "xbee/random.h"

#include "psa/crypto.h"

int xbee_random(void* output, size_t output_len)
{
    psa_status_t status = psa_generate_random(output, output_len);

    if (status != PSA_SUCCESS) {
        return -EIO;
    }

    return 0;
}
