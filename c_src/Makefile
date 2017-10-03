# Based on c_src.mk from erlang.mk by Loic Hoguin <essen@ninenines.eu>

CURDIR := $(shell pwd)
BASEDIR := $(abspath $(CURDIR)/..)

PROJECT ?= $(notdir $(BASEDIR))
PROJECT := $(strip $(PROJECT))

ERTS_INCLUDE_DIR ?= $(shell erl -noshell -s init stop -eval "io:format(\"~s/erts-~s/include/\", [code:root_dir(), erlang:system_info(version)]).")
ERL_INTERFACE_INCLUDE_DIR ?= $(shell erl -noshell -s init stop -eval "io:format(\"~s\", [code:lib_dir(erl_interface, include)]).")
ERL_INTERFACE_LIB_DIR ?= $(shell erl -noshell -s init stop -eval "io:format(\"~s\", [code:lib_dir(erl_interface, lib)]).")

C_SRC_DIR = $(CURDIR)
C_SRC_OUTPUT ?= $(CURDIR)/../priv/$(PROJECT).so

# System type and C compiler/flags.

UNAME_SYS := $(shell uname -s)
ifeq ($(UNAME_SYS), Darwin)
	CC ?= cc
	CFLAGS ?= -O3 -std=c99 -arch x86_64 -finline-functions -Wall -Wmissing-prototypes
	CXXFLAGS ?= -O3 -arch x86_64 -finline-functions -Wall
	LDFLAGS ?= -arch x86_64 -flat_namespace -undefined suppress
else ifeq ($(UNAME_SYS), FreeBSD)
	CC ?= cc
	CFLAGS ?= -O3 -std=c99 -finline-functions -Wall -Wmissing-prototypes
	CXXFLAGS ?= -O3 -finline-functions -Wall
else ifeq ($(UNAME_SYS), Linux)
	CC ?= gcc
	CXX ?= g++
	CFLAGS ?= -O3 -std=c99 -finline-functions -Wall -Wmissing-prototypes
	CXXFLAGS ?= -O3 -finline-functions -Wall
endif

CFLAGS += -g3 -fPIC -I $(ERTS_INCLUDE_DIR) -I $(ERL_INTERFACE_INCLUDE_DIR)
CXXFLAGS += -g3 -fPIC -I $(ERTS_INCLUDE_DIR) -I $(ERL_INTERFACE_INCLUDE_DIR) -I $(BASEDIR)/xdelta/xdelta3/
# hacks
CXXFLAGS += -Wwrite-strings -Wliteral-suffix

LDLIBS += -L $(ERL_INTERFACE_LIB_DIR) -lerl_interface -lei -lstdc++ \
 	  -L $(BASEDIR)/xdelta/xdelta3/

LDFLAGS += -shared -rdynamic


# Verbosity.

c_verbose_0 = @echo " C     " $(?F);
c_verbose = $(c_verbose_$(V))

cpp_verbose_0 = @echo " CPP   " $(?F);
cpp_verbose = $(cpp_verbose_$(V))

link_verbose_0 = @echo " LD    " $(@F);
link_verbose = $(link_verbose_$(V))

SOURCES := $(shell find $(C_SRC_DIR) -type f \( -name "*.c" -o -name "*.C" -o -name "*.cc" -o -name "*.cpp" \))
OBJECTS = $(addsuffix .o, $(basename $(SOURCES)))

COMPILE_C = $(c_verbose) $(CC) $(CFLAGS) $(CPPFLAGS) -c
COMPILE_CPP = $(cpp_verbose) $(CXX) $(CXXFLAGS) $(CPPFLAGS) -c

$(C_SRC_OUTPUT):  $(BASEDIR)/xdelta/xdelta3/build $(OBJECTS)
	@mkdir -p $(BASEDIR)/priv/
	$(link_verbose) $(CC) $(OBJECTS) \
		-DXD3_MAIN=0 \
        $(BASEDIR)/xdelta/xdelta3/xdelta3-xdelta3.o \
		$(LDFLAGS) $(LDLIBS) -o $(C_SRC_OUTPUT)


$(BASEDIR)/xdelta:
	cd $(BASEDIR) \
	    && git clone "git://github.com/arcusfelis/xdelta.git" \
		&& cd xdelta \
	    && git checkout merger

$(BASEDIR)/xdelta/xdelta3/build: $(BASEDIR)/xdelta
	cd $(BASEDIR)/xdelta/xdelta3/ \
      && ./generate_build_files.sh
	mkdir -p $(BASEDIR)/xdelta/xdelta3/build
	cd $(BASEDIR)/xdelta/xdelta3/ \
	  && ./configure \
	  	--disable-dependency-tracking \
		"CXXFLAGS=-g3 -fPIC " \
		"CFLAGS=-g3 -fPIC " \
		--enable-static \
		--disable-shared \
		--with-liblzma=no \
		--prefix=$(pwd)/build
	# Something is wrong with the asserts:
	# define error: expected declaration specifiers or ‘...’ before numeric constant
	sed -i '/static_assert/d' $(BASEDIR)/xdelta/xdelta3/xdelta3.h
	cd $(BASEDIR)/xdelta/xdelta3/ \
	  && make


%.o: %.c
	$(COMPILE_C) $(OUTPUT_OPTION) $<

%.o: %.cc
	$(COMPILE_CPP) $(OUTPUT_OPTION) $<

%.o: %.C
	$(COMPILE_CPP) $(OUTPUT_OPTION) $<

%.o: %.cpp
	$(COMPILE_CPP) $(OUTPUT_OPTION) $<

clean:
	@rm -f $(C_SRC_OUTPUT) $(OBJECTS)
	@rm -rf "$(BASEDIR)/open-xdelta3" "$(BASEDIR)/xdelta3_build" "$(BASEDIR)/xdelta3_install"
