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

#include <string>
#include <cstring>

#include <erl_nif.h>
#include <google/vcencoder.h>

#define BADARG             enif_make_badarg(env)

static ERL_NIF_TERM
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
vcdiff_load(ErlNifEnv *env, void **priv_data, ERL_NIF_TERM load_info)
{
    return 0;
}

static ErlNifFunc vcdiff_exports[] = {
    {"vcdiff_encode", 2, vcdiff_encode},
};

ERL_NIF_INIT(vcdiff_nif, vcdiff_exports, vcdiff_load, NULL, NULL, NULL)
