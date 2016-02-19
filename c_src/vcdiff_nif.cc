/*
 * Copyright (c) 2013 Jachym Holecek <freza@circlewave.net>
 * Copyright (c) 2014 Jihyun Yu <yjh0502@gmail.com>
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
#include <google/vcencoder.h>
#include <string>
#include <string.h>

/*
 * Type name normalization, utility macros.
 */

typedef unsigned int         uint_t;
typedef unsigned long         ulong_t;
typedef ErlNifEnv         nif_heap_t;
typedef ERL_NIF_TERM         nif_term_t;
typedef ErlNifFunc         nif_func_t;
typedef ErlNifMutex         nif_lock_t;
typedef ErlNifCond         nif_cond_t;
typedef ErlNifResourceType     nif_type_t;
typedef ErlNifBinary         nif_bin_t;
typedef ErlNifTid         nif_tid_t;
typedef ErlNifPid         nif_pid_t;

#define BADARG             enif_make_badarg(env)

static nif_term_t
vcdiff_encode(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[])
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

    std::string output;
    open_vcdiff::VCDiffEncoder encoder((char *)dic.data, dic.size);
    encoder.SetFormatFlags(open_vcdiff::VCD_FORMAT_INTERLEAVED | open_vcdiff::VCD_FORMAT_CHECKSUM);
    encoder.Encode((char *)target.data, target.size, &output);

    if (!enif_alloc_binary(output.size(), &encoded)) {
        return (BADARG);
    }
    memcpy(encoded.data, &output[0], output.size());

    return enif_make_binary(env, &encoded);
}

static int
vcdiff_load(nif_heap_t *env, void **priv_data, nif_term_t load_info)
{
    return 0;
}

static nif_func_t vcdiff_exports[] = {
    {"vcdiff_encode", 2, vcdiff_encode},
};

ERL_NIF_INIT(vcdiff_nif, vcdiff_exports, vcdiff_load, NULL, NULL, NULL)
