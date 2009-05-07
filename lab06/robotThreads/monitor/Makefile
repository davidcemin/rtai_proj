#threads Makefile. It should be the last to be called
TARGET = 

#flags
CFLAGS   = -O2 -Wall
CINCLUDE = -I. -I../../simulCalc/ -I../../../include/
CLIBDIR  =
CLIBS    = -lpthread 
DEPFLAGS = -MM

#includes
INCLUDE    = ${CINCLUDE}
FLAGS      = ${CFLAGS}
LIBDIR     = ${CLIBDIR}
LIBS       = ${CLIBS}
LIBINCLUDE = ../../../lib

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
LIBFILES = $(LIBINCLUDE)/libRobot.a

#General
QUIET = @

#RTAI
prefix := $(shell rtai-config --prefix)

ifeq ($(prefix),)
$(error Please add <rtai-install>/bin to your PATH variable)
endif

LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

.PHONY: clean install distclean clobber install

all: $(OBJECTS)

# The variable $@ has the value of the target and $^ has the value of the dependencies
# $< has the value of the first dependency file

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
	$(QUIET)rm -f $(TARGET) *.d *.a *.gch

clobber: distclean
# DO NOT DELETE
