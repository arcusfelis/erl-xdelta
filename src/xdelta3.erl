-module(xdelta3).

-export([encode/2]).

encode(Dict, Data) ->
    xdelta3_nif:xdelta3_encode(Dict, Data).
