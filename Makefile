# ========================================================================================
# Makefile for airspy_tcp
# ========================================================================================

# ========================================================================================
# Compile flags

CC = gcc
COPT = -O3 -march=native
CFLAGS = -Wall -Wextra -Wpedantic -Werror -std=gnu11 -D_GNU_SOURCE
CFLAGS += -D BUILD_VERSION="\"$(shell git describe --dirty --always)\""	\
		-D BUILD_DATE="\"$(shell date '+%Y-%m-%d_%H:%M:%S')\""

BIN = airspy_tcp

# ========================================================================================
# Source files

SRCDIR = ./

SRC = $(SRCDIR)/airspy_tcp.c

# ========================================================================================
# External Libraries

LIBSDIR = 

LIBS = -lm -pthread `pkg-config --libs libairspy` -lusb-1.0

CFLAGS += `pkg-config --cflags libairspy`

# ========================================================================================
# Makerules

all:
	$(CC) $(COPT) $(CFLAGS) $(SRC) -o $(BIN) $(LIBSDIR) $(LIBS)

debug: COPT = -Og -ggdb -fno-omit-frame-pointer -D__DEBUG
debug: all

clean:
	rm -fv $(BIN)
