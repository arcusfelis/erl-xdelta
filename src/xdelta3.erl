-module(xdelta3).

-export([encode/2]).
-export([decode/2]).
-export([merge/1]).

encode(Source, Target) ->
    xdelta3_nif:xdelta3_encode(Source, Target).

decode(Source, Delta) ->
    xdelta3_nif:xdelta3_decode(Source, Delta).

merge(List) ->
    xdelta3_nif:xdelta3_merge(List).
