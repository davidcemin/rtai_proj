#Main Makefile

#defines
CONTROLDIR := control
SIMULDIR := simulation

#COMMON
LIBDIR  := lib
COMMON  := common
INCLUDE	:= include
UTIL 	:= $(LIBDIR) $(COMMON) 

#CONTROL
CONTROLTHREAD	:= $(CONTROLDIR)/controlThreads
CONTROLMONITOR 	:= $(CONTROLTHREAD)/monitor
CONTROL	:= $(CONTROLMONITOR) $(CONTROLTHREAD) $(CONTROLDIR)

#SIMULATION
SIMULTHREAD	:= $(SIMULDIR)/simThreads
SIMUL := $(SIMULTHREAD) $(SIMULDIR)

#General
QUIET = @

#All dirs
ALLDIRS := $(UTIL) $(CONTROL) $(SIMUL)

.PHONY: all clean clobber doc 

all: 
	$(QUIET)for i in $(ALLDIRS) ; do cd $$i; $(MAKE); cd -;done

clean:
	$(QUIET)for i in $(ALLDIRS); do cd $$i; $(MAKE) $@; cd -;  done

clobber: clean
	$(QUIET)for i in $(ALLDIRS); do cd $$i; $(MAKE) $@ ; cd -; done
	$(QUIET)cd $(INCLUDE); rm *.gch; cd -;
	$(QUIET) rm *.dat 

doc:
	$(QUIET)doxygen Doxyfile

