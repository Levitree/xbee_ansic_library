/*
 * Secure Remote Password 6a implementation
 * Copyright (c) 2010 Tom Cocagne. All rights reserved.
 * https://github.com/cocagne/csrp
 *
 * Modified by Digi International to use Mbed TLS instead of OpenSSL.
 * Copyright (c) 2019 Digi International Inc.
 * All rights not expressly granted are reserved.
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Tom Cocagne
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*
 * SRP functionality is stubbed out — not used in this firmware.
 */

#include <stddef.h>
#include "util/srp.h"

struct SRPVerifier { int dummy; };
struct SRPUser { int dummy; };

int srp_create_salted_verification_key(const char* username,
    const unsigned char* password, int len_password,
    const unsigned char** bytes_s, int* len_s,
    const unsigned char** bytes_v, int* len_v)
{
    (void)username; (void)password; (void)len_password;
    (void)bytes_s; (void)len_s; (void)bytes_v; (void)len_v;
    return -1;
}

struct SRPVerifier* srp_verifier_new(const char* username,
    const unsigned char* bytes_s, int len_s,
    const unsigned char* bytes_v, int len_v,
    const unsigned char* bytes_A, int len_A,
    const unsigned char** bytes_B, int* len_B)
{
    (void)username; (void)bytes_s; (void)len_s;
    (void)bytes_v; (void)len_v; (void)bytes_A; (void)len_A;
    (void)bytes_B; (void)len_B;
    return NULL;
}

void srp_verifier_delete(struct SRPVerifier* ver)
{
    (void)ver;
}

int srp_verifier_is_authenticated(struct SRPVerifier* ver)
{
    (void)ver;
    return 0;
}

const char* srp_verifier_get_username(struct SRPVerifier* ver)
{
    (void)ver;
    return NULL;
}

const unsigned char* srp_verifier_get_session_key(struct SRPVerifier* ver, int* key_length)
{
    (void)ver;
    if (key_length) *key_length = 0;
    return NULL;
}

int srp_verifier_get_session_key_length(struct SRPVerifier* ver)
{
    (void)ver;
    return 0;
}

void srp_verifier_verify_session(struct SRPVerifier* ver,
    const unsigned char* user_M, const unsigned char** bytes_HAMK)
{
    (void)ver; (void)user_M;
    *bytes_HAMK = NULL;
}

struct SRPUser* srp_user_new(const char* username,
    const unsigned char* bytes_password, int len_password)
{
    (void)username; (void)bytes_password; (void)len_password;
    return NULL;
}

void srp_user_delete(struct SRPUser* usr)
{
    (void)usr;
}

int srp_user_is_authenticated(struct SRPUser* usr)
{
    (void)usr;
    return 0;
}

const char* srp_user_get_username(struct SRPUser* usr)
{
    (void)usr;
    return NULL;
}

const unsigned char* srp_user_get_session_key(struct SRPUser* usr, int* key_length)
{
    (void)usr;
    if (key_length) *key_length = 0;
    return NULL;
}

int srp_user_get_session_key_length(struct SRPUser* usr)
{
    (void)usr;
    return 0;
}

int srp_user_start_authentication(struct SRPUser* usr, const char** username,
    const unsigned char** bytes_A, int* len_A)
{
    (void)usr; (void)username; (void)bytes_A; (void)len_A;
    return -1;
}

int srp_user_process_challenge(struct SRPUser* usr,
    const unsigned char* bytes_s, int len_s,
    const unsigned char* bytes_B, int len_B,
    const unsigned char** bytes_M, int* len_M)
{
    (void)usr; (void)bytes_s; (void)len_s;
    (void)bytes_B; (void)len_B; (void)bytes_M; (void)len_M;
    return -1;
}

void srp_user_verify_session(struct SRPUser* usr, const unsigned char* bytes_HAMK)
{
    (void)usr; (void)bytes_HAMK;
}
