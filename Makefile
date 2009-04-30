#Main Makefile

#defines
MAINDIR = lab05
LIBDIR  = lib
THREAD  = $(MAINDIR)/robotThreads
SIMUL	= $(MAINDIR)/simulCalc
INCLUDE = include
DIRS = $(LIBDIR) $(SIMUL) $(THREAD) $(MAINDIR)

#General
QUIET = @

.PHONY: all clean install distclean clobber doc

all:
	$(QUIET)for i in $(DIRS); do cd $$i; $(MAKE); cd -;done

clean:
	$(QUIET)for i in $(DIRS); do cd $$i; $(MAKE) $@ ; cd -; done

install:

distclean: clean
	$(QUIET)for i in $(DIRS); do cd $$i; $(MAKE) $@ ; cd -; done
	$(QUIET)cd $(INCLUDE); rm *.gch; cd -;

clobber: distclean
	$(QUIET) rm *.dat 

doc:
	$(QUIET)doxygen Doxyfile

# DO NOT DELETE
