/*
 * Copyright (c) 2016 Jihyun Yu <yjh0502@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <erl_nif.h>
#include "config.h"


//#include "xdelta3.c"
extern "C"
{
#include "xdelta3.h"
#include "xdelta3-internal.h"
}

//#include "xdelta3-decode.h"
//#include "xdelta3-merge.h"
//#include "xdelta3-list.h"
//#include "xdelta3-internal.h"

#define BADARG             enif_make_badarg(env)

static ERL_NIF_TERM
xdelta3_encode(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[])
{
    ErlNifBinary dic, target, encoded;

    if (argc != 2) {
        return (BADARG);
    }

    if (!enif_inspect_binary(env, argv[0], &dic)) {
        return (BADARG);
    }
    if (!enif_inspect_binary(env, argv[1], &target)) {
        return (BADARG);
    }

    uint8_t *from_buf = NULL, *to_buf = NULL, *delta_buf = NULL;
    size_t from_len = 0, to_len, delta_alloc, delta_size = 0;
    int flags;
    int level;
    level = 1;
    flags = (level << XD3_COMPLEVEL_SHIFT) & XD3_COMPLEVEL_MASK;
    from_buf = (uint8_t*) dic.data;
    from_len = (size_t)   dic.size;

    to_buf = (uint8_t*) target.data;
    to_len = (size_t)   target.size;

    delta_alloc = to_len * 11 / 10;

    if (!enif_alloc_binary(delta_alloc, &encoded)) {
        return (BADARG);
    }
    delta_buf = (uint8_t*) encoded.data;

    int ret = xd3_encode_memory(to_buf, to_len, from_buf, from_len,
                    delta_buf, &delta_size, delta_alloc, flags);
    if (ret != 0) {
      enif_release_binary(&encoded);
      // xd3_strerror(ret)
      return (BADARG);
    }

    if (!enif_realloc_binary(&encoded, delta_size)) {
        enif_release_binary(&encoded);
        return (BADARG);
    }
    return enif_make_binary(env, &encoded);
}


static ERL_NIF_TERM
xdelta3_merge(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[])
{
    ErlNifBinary delta1, delta2;

    if (argc != 2) {
        return (BADARG);
    }

    if (!enif_inspect_binary(env, argv[0], &delta1)) {
        return (BADARG);
    }
    if (!enif_inspect_binary(env, argv[1], &delta2)) {
        return (BADARG);
    }

    xd3_main_cmdline ();

}

static int
xdelta3_load(ErlNifEnv *env, void **priv_data, ERL_NIF_TERM load_info)
{
    return 0;
}

static ErlNifFunc xdelta3_exports[] = {
    {"xdelta3_encode", 2, xdelta3_encode},
    {"xdelta3_merge", 2, xdelta3_merge},
};

ERL_NIF_INIT(xdelta3_nif, xdelta3_exports, xdelta3_load, NULL, NULL, NULL)
