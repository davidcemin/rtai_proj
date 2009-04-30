#lib Makefile
TARGET = libRobot.a

#flags
CFLAGS	 =-O2 -Wall
CINCLUDE =-I../include/
CLIBDIR	 =
CLIBS	 =-lm
DEPFLAGS = -MM

#includes
INCLUDE	= ${CINCLUDE}
FLAGS	= ${CFLAGS}
LIBDIR	= ${CLIBDIR}
LIBS	= ${CLIBS}

#Setzen alle zusammen..
CMP	= gcc
LD 	= ld
AR 	= ar
ARFLAGS	 = -crus
CMPFLAGS = ${FLAGS} ${INCLUDE}
LDFLAGS	 = ${LIBDIR} ${LIBS}

#Special variables
CSOURCES = $(wildcard *.c) 
DEPS = $(CSOURCES:.c=.d)
OBJS = $(CSOURCES:.c=.o)

#General
QUIET = @

.PHONY: clean install distclean clobber install

all: $(TARGET)

$(TARGET): $(OBJS)
	$(QUIET)echo Creating lib: $@
	$(QUIET)$(AR) $(ARFLAGS) $@ $^
#$(QUIET)ranlib $@

$(OBJS): $(CSOURCES)
	$(QUIET)echo Creating object: $@
	$(QUIET)$(CMP) $(CMPFLAGS) $(LIBS) -c $^

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
	$(QUIET)rm -f $(TARGET) *.d *.a

clobber: distclean
# DO NOT DELETE
