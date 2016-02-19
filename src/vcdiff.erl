-module(vcdiff).

-export([encode/2]).

encode(Dict, Data) ->
    vcdiff_nif:vcdiff_encode(Dict, Data).
