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
#include "xdelta3-merger.h"
}

//#include "xdelta3-decode.h"
//#include "xdelta3-merge.h"
//#include "xdelta3-list.h"
//#include "xdelta3-internal.h"

#define BADARG             enif_make_badarg(env)

ERL_NIF_TERM make_error_term(ErlNifEnv *env, int ret);

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
    flags = ((level << XD3_COMPLEVEL_SHIFT) & XD3_COMPLEVEL_MASK);
    from_buf = (uint8_t*) dic.data;
    from_len = (size_t)   dic.size;

    to_buf = (uint8_t*) target.data;
    to_len = (size_t)   target.size;

    delta_alloc = to_len * 11 / 10 + 10000;

    if (!enif_alloc_binary(delta_alloc, &encoded)) {
        return (BADARG);
    }
    delta_buf = (uint8_t*) encoded.data;

    int ret = xd3_encode_memory(to_buf, to_len, from_buf, from_len,
                    delta_buf, &delta_size, delta_alloc, flags);
    if (ret != 0) {
      enif_release_binary(&encoded);
      return make_error_term(env, ret);
    }

    if (!enif_realloc_binary(&encoded, delta_size)) {
        enif_release_binary(&encoded);
        return (BADARG);
    }
    return enif_make_binary(env, &encoded);
}

static ERL_NIF_TERM
xdelta3_decode(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[])
{
    ErlNifBinary source, delta, encoded;

    if (argc != 2) {
        return (BADARG);
    }

    if (!enif_inspect_binary(env, argv[0], &source)) {
        return (BADARG);
    }
    if (!enif_inspect_binary(env, argv[1], &delta)) {
        return (BADARG);
    }

    uint8_t *from_buf = NULL, *input_buf = NULL, *output_buf = NULL;
    size_t from_len = 0, input_len, output_alloc, output_size = 0;
    int flags;
    flags = 0;
    from_buf = (uint8_t*) source.data;
    from_len = (size_t)   source.size;

    input_buf = (uint8_t*) delta.data;
    input_len = (size_t)   delta.size;

    output_alloc = (input_len + from_len) * 11 / 10 + 10000;

    if (!enif_alloc_binary(output_alloc, &encoded)) {
        return (BADARG);
    }
    output_buf = (uint8_t*) encoded.data;

    int ret = xd3_decode_memory(input_buf, input_len, from_buf, from_len,
                    output_buf, &output_size, output_alloc, flags);
    if (ret != 0) {
      enif_release_binary(&encoded);

      return enif_raise_exception(env, make_error_term(env, ret));
    }

    if (!enif_realloc_binary(&encoded, output_size)) {
        enif_release_binary(&encoded);
        return (BADARG);
    }
    return enif_make_binary(env, &encoded);
}


ERL_NIF_TERM make_error_term(ErlNifEnv *env, int ret)
{
    ErlNifBinary bin;
    const char* err_description = xd3_strerror(ret);
    unsigned err_description_length = strlen(err_description);

    if (!enif_alloc_binary(err_description_length, &bin)) {
        return (BADARG);
    }

    memcpy(bin.data, err_description, err_description_length);

    return enif_make_binary(env, &bin);
}


ERL_NIF_TERM xd3_merge (ErlNifEnv* env, ERL_NIF_TERM merge_terms)
{
    unsigned merge_terms_length;
    unsigned int err = 0;
    xd3_merger* merger;

    if (!enif_get_list_length(env, merge_terms, &merge_terms_length))
       return enif_make_badarg(env);

    if (merge_terms_length < 2)
       return enif_make_badarg(env);

    int level;
    level = 1;
    int flags = ((level << XD3_COMPLEVEL_SHIFT) & XD3_COMPLEVEL_MASK);
    merger = xd3_merger_init(flags);
    if (!merger)
       return enif_make_badarg(env);

    ERL_NIF_TERM head;
    ERL_NIF_TERM tail;
    ErlNifBinary bin;
    for (unsigned i = 0; i < merge_terms_length; i++)
    {
        enif_get_list_cell(env, merge_terms, &head, &tail);

        if (!enif_inspect_binary(env, head, &bin))
        {
            err = 1;
            break;
        }
        err = xd3_merger_add_input(merger, (char*) bin.data, bin.size);
        if (err)
            break;
        merge_terms = tail;
    }


    if (!err)
        err = xd3_merger_run(merger);

    if (!err)
    {
       ErlNifBinary result;
       size_t output_size;
       char* output_data;

       output_size = xd3_merger_get_output_size(merger);
       output_data = xd3_merger_get_output_data(merger);

       if (!enif_alloc_binary(output_size, &result)) {
           xd3_merger_clean(merger);
           return enif_make_binary(env, &result);
       }
       memcpy(result.data, output_data, output_size);
       xd3_merger_clean(merger);
       return enif_make_binary(env, &result);
    }
    else
    {
       xd3_merger_clean(merger);
       return enif_make_badarg(env);
    }
}


static ERL_NIF_TERM
xdelta3_merge(ErlNifEnv *env, int argc, const ERL_NIF_TERM argv[])
{
    if (argc != 1) {
        return (BADARG);
    }

    return xd3_merge(env, argv[0]);

}

static int
xdelta3_load(ErlNifEnv *env, void **priv_data, ERL_NIF_TERM load_info)
{
    return 0;
}

static ErlNifFunc xdelta3_exports[] = {
    {"xdelta3_encode", 2, xdelta3_encode},
    {"xdelta3_decode", 2, xdelta3_decode},
    {"xdelta3_merge", 1, xdelta3_merge},
};

ERL_NIF_INIT(xdelta3_nif, xdelta3_exports, xdelta3_load, NULL, NULL, NULL)
