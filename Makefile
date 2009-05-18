#Main Makefile

#defines
MAINDIR = control
LIBDIR  = lib
THREAD  = $(MAINDIR)/robotThreads
SIMUL	= $(MAINDIR)/simulCalc
INCLUDE = include
MONITOR = $(THREAD)/monitor
DIRS = $(LIBDIR) $(SIMUL) $(MONITOR) $(THREAD) $(MAINDIR)

#General
QUIET = @

.PHONY: all clean install distclean clobber doc

all:
	$(QUIET)for i in $(DIRS); do cd $$i; $(MAKE); cd -;done

clean:
	$(QUIET)for i in $(DIRS); do cd $$i; $(MAKE) $@; cd -;  done

install:

clobber: clean
	$(QUIET)for i in $(DIRS); do cd $$i; $(MAKE) $@ ; cd -; done
	$(QUIET)cd $(INCLUDE); rm *.gch; cd -;
	$(QUIET) rm *.dat 

doc:
	$(QUIET)doxygen Doxyfile

# DO NOT DELETE
