-module(vcdiff).

-export([encode/2, encode_gz/2]).

encode(Dict, Data) ->
    vcdiff_nif:vcdiff_encode(Dict, Data).

encode_gz(Dict, Data) ->
    zlib:gzip(encode(Dict, Data)).

