#Main Makefile

#defines
CONTROLDIR := control
LIBDIR  := lib
COMMON  := common
THREAD  := $(CONTROLDIR)/controlThreads
INCLUDE := include
MONITOR := $(THREAD)/monitor
DIRS = $(LIBDIR) $(COMMON) $(MONITOR) $(THREAD) $(CONTROLDIR)

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
