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

CFLAGS += -fPIC -I $(ERTS_INCLUDE_DIR) -I $(ERL_INTERFACE_INCLUDE_DIR)
CXXFLAGS += -fPIC -I $(ERTS_INCLUDE_DIR) -I $(ERL_INTERFACE_INCLUDE_DIR) -I $(BASEDIR)/xdelta3_install/include/

LDLIBS += -L $(ERL_INTERFACE_LIB_DIR) -lerl_interface -lei -lstdc++ \
		  -L $(BASEDIR)/xdelta3_install/lib/ \
		  -l:libvcdcom.a \
		  -l:libvcddec.a \
		  -l:libvcdenc.a

LDFLAGS += -shared -rdynamic
# Link xdelta3 library statically
LDFLAGS += -Wl,--whole-archive \
		   $(BASEDIR)/xdelta3_install/lib/libvcdcom.a \
		   $(BASEDIR)/xdelta3_install/lib/libvcdenc.a \
		   $(BASEDIR)/xdelta3_install/lib/libvcddec.a \
		   -Wl,--no-whole-archive

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

$(C_SRC_OUTPUT): $(BASEDIR)/xdelta3_install $(OBJECTS)
	@mkdir -p $(BASEDIR)/priv/
	$(link_verbose) $(CC) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(C_SRC_OUTPUT)

$(BASEDIR)/priv/libvcdenc.so.0:
	@mkdir -p $(BASEDIR)/priv/
	cp $(wildcard $(BASEDIR)/xdelta3_install/lib/*) $(BASEDIR)/priv/

$(BASEDIR)/open-xdelta3:
	cd $(BASEDIR) \
		&& git clone "git://github.com/google/open-xdelta3.git"
	cd $(BASEDIR)/open-xdelta3 \
		&& git submodule update --init --recursive

$(BASEDIR)/xdelta3_build:
	mkdir -p $(BASEDIR)/xdelta3_build

$(BASEDIR)/xdelta3_build/Makefile: $(BASEDIR)/xdelta3_build $(BASEDIR)/open-xdelta3
	cat $(BASEDIR)/open-xdelta3/CMakeLists.txt > $(BASEDIR)/open-xdelta3/CMakeLists.txt.tmp
	echo "set(CMAKE_POSITION_INDEPENDENT_CODE ON)" > $(BASEDIR)/open-xdelta3/CMakeLists.txt
	cat $(BASEDIR)/open-xdelta3/CMakeLists.txt.tmp >> $(BASEDIR)/open-xdelta3/CMakeLists.txt
	rm $(BASEDIR)/open-xdelta3/CMakeLists.txt.tmp
	cd $(BASEDIR)/xdelta3_build && cmake  \
		-DCMAKE_INSTALL_PREFIX=$(BASEDIR)/xdelta3_install \
		-DBUILD_STATIC_LIBS=ON \
		-Dxdelta3_build_test=OFF \
	   	-Dxdelta3_build_exec=OFF ../open-xdelta3

$(BASEDIR)/xdelta3_install: $(BASEDIR)/xdelta3_build/Makefile
	cd $(BASEDIR)/xdelta3_build && make && make install

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
