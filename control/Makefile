#lab04 Makefile. It should be the last to be called
TARGET := control

########################################################################
#Include dirs
THREAD = ./controlThreads
MONITOR = $(THREAD)/monitor
COMMON = ../common
INCLUDE = ../include

########################################################################
#flags
CFLAGS   := -O2 -Wall
CINCLUDE := -I$(INCLUDE) -I$(THREAD) -I$(COMMON) -I$(MONITOR)
CLIBDIR  :=
CLIBS    := -lm -lpthread
DEPFLAGS := -MM

########################################################################
#includes
INCLUDE    = ${CINCLUDE}
FLAGS      = ${CFLAGS}
LIBDIR     = ${CLIBDIR}
LIBS       = ${CLIBS}
LIBINCLUDE = ../lib

########################################################################
#Setzen alle zusammen..
CMP      = gcc
LD       = ld
AR       = ar
ARFLAGS  = rvu
CMPFLAGS = ${FLAGS} ${INCLUDE}
LDFLAGS  = ${LIBS}

########################################################################
CSOURCES = $(wildcard *.c)
OBJECTS  = $(CSOURCES:.c=.o)
DEPS     = $(CSOURCES:.c=.d)
LIBFILES = $(COMMON)/*.o $(MONITOR)/*.o $(THREAD)/*.o $(LIBINCLUDE)/*.a 

########################################################################
#RTAI
prefix := $(shell rtai-config --prefix)

ifeq ($(prefix),)
$(error Please add <rtai-install>/bin to your PATH variable)
endif

LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

########################################################################

LIBFILES += $(LXRT_LDFLAGS)

#General
QUIET = @

.PHONY: clean clobber 

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(QUIET)echo Compiling executable file: $@
	$(QUIET)${CMP} $(LDFLAGS) -o $@  $^ $(LIBFILES) $(LXRT_LDFLAGS)

%.o: %.c
	$(QUIET)echo Creating object: $@
	$(QUIET)$(CMP) $(CMPFLAGS) $(LIBS) -c $(LXRT_CFLAGS) $^

clean:
	$(QUIET)echo Removing temp and object files
	$(QUIET)rm -f *~ *.bak *.o


clobber: clean
	$(QUIET)echo Removing executable and some unnecessary files
	$(QUIET)rm -f $(TARGET) $(DEPS) 
	$(QUIET) rm -f *.gch

-include $(DEPS)
%.d: %.c
	$(QUIET)echo Creating dep: $@
	$(QUIET)$(CMP) $(DEPFLAGS) $(LIBS) $(CMPFLAGS) $< \
	| sed 's!\($*\)\.o[ :]*!\1.o $@ : !g' > $@; \
	[ -s $@ ] || rm -f $@

