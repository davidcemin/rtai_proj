#lab04 Makefile. It should be the last to be called
TARGET := robotSim

#Include dirs
ROBOTHREAD = ./robotThreads
SIMUL = ./simulCalc

#flags
CFLAGS   := -O2 -Wall
CINCLUDE := -I../include/ -I$(ROBOTHREAD) -I$(SIMULCALC)
CLIBDIR  :=
CLIBS    := -lm -lpthread
DEPFLAGS := -MM

#includes
INCLUDE    = ${CINCLUDE}
FLAGS      = ${CFLAGS}
LIBDIR     = ${CLIBDIR}
LIBS       = ${CLIBS}
LIBINCLUDE = ../lib

#Setzen alle zusammen..
CMP      = gcc
LD       = ld
AR       = ar
ARFLAGS  = rvu
CMPFLAGS = ${FLAGS} ${INCLUDE}
LDFLAGS  = ${LIBS}

CSOURCES = $(wildcard *.c)
OBJECTS  = $(CSOURCES:.c=.o)
DEPS     = $(CSOURCES:.c=.d)
LIBFILES = $(SIMUL)/simulCalcsUtils.o $(ROBOTHREAD)/robotThreads.o $(LIBINCLUDE)/libRobot.a 


#RTAI
prefix := $(shell rtai-config --prefix)

ifeq ($(prefix),)
$(error Please add <rtai-install>/bin to your PATH variable)
endif

LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

LIBFILES += $(LXRT_LDFLAGS)

#General
QUIET = @

.PHONY: clean install distclean clobber install


all: $(TARGET)

# The variable $@ has the value of the target and $^ has the value of the dependencies
# $< has the value of the first dependency file
$(TARGET): $(OBJECTS)
	$(QUIET)echo Compiling executable file: $@
	$(QUIET)${CMP} $(LDFLAGS) -o $@  $^ $(LIBFILES) $(LXRT_LDFLAGS)

$(OBJECTS): $(CSOURCES)
	$(QUIET)echo Creating object: $@
	$(QUIET)$(CMP) $(CMPFLAGS) $(LIBS) -c $(LXRT_CFLAGS) $^

-include $(DEPS)
%.d: %.c
	$(QUIET)echo Creating dep: $@
	$(QUIET)$(CMP) $(DEPFLAGS) $(LIBS) $(CMPFLAGS) $< \
	| sed 's!\($*\)\.o[ :]*!\1.o $@ : !g' > $@; \
	[ -s $@ ] || rm -f $@

clean:
	$(QUIET)echo Removing temp and object files
	$(QUIET)rm -f *~ *.bak *.o

install:

distclean: clean
	$(QUIET)echo Removing executable and some unnecessary files
	$(QUIET)rm -f $(TARGET) $(DEPS) 
	$(QUIET) rm -f *.gch

clobber: distclean
# DO NOT DELETE
