#
# This is a simple wrapper Makefile that calls the main Makefile.perf
# with a -j option to do parallel builds
#
# If you want to invoke the perf build in some non-standard way then
# you can use the 'make -f Makefile.perf' method to invoke it.
#

#
# Clear out the built-in rules GNU make defines by default (such as .o targets),
# so that we pass through all targets to Makefile.perf:
#
.SUFFIXES:

#
# We don't want to pass along options like -j:
#
<<<<<<< HEAD
unexport MAKEFLAGS
=======
PROGRAMS += $(OUTPUT)perf

LANG_BINDINGS =

# what 'all' will build and 'install' will install, in perfexecdir
ALL_PROGRAMS = $(PROGRAMS) $(SCRIPTS)

# what 'all' will build but not install in perfexecdir
OTHER_PROGRAMS = $(OUTPUT)perf

# Set paths to tools early so that they can be used for version tests.
ifndef SHELL_PATH
	SHELL_PATH = /bin/sh
endif
ifndef PERL_PATH
	PERL_PATH = /usr/bin/perl
endif

export PERL_PATH

$(OUTPUT)util/parse-events-flex.c: util/parse-events.l $(OUTPUT)util/parse-events-bison.c
	$(QUIET_FLEX)$(FLEX) --header-file=$(OUTPUT)util/parse-events-flex.h $(PARSER_DEBUG_FLEX) -t util/parse-events.l > $(OUTPUT)util/parse-events-flex.c

$(OUTPUT)util/parse-events-bison.c: util/parse-events.y
	$(QUIET_BISON)$(BISON) -v util/parse-events.y -d $(PARSER_DEBUG_BISON) -o $(OUTPUT)util/parse-events-bison.c -p parse_events_

$(OUTPUT)util/pmu-flex.c: util/pmu.l $(OUTPUT)util/pmu-bison.c
	$(QUIET_FLEX)$(FLEX) --header-file=$(OUTPUT)util/pmu-flex.h -t util/pmu.l > $(OUTPUT)util/pmu-flex.c

$(OUTPUT)util/pmu-bison.c: util/pmu.y
	$(QUIET_BISON)$(BISON) -v util/pmu.y -d -o $(OUTPUT)util/pmu-bison.c -p perf_pmu_

$(OUTPUT)util/parse-events.o: $(OUTPUT)util/parse-events-flex.c $(OUTPUT)util/parse-events-bison.c
$(OUTPUT)util/pmu.o: $(OUTPUT)util/pmu-flex.c $(OUTPUT)util/pmu-bison.c

LIB_FILE=$(OUTPUT)libperf.a

LIB_H += ../../include/uapi/linux/perf_event.h
LIB_H += ../../include/linux/rbtree.h
LIB_H += ../../include/linux/list.h
LIB_H += ../../include/uapi/linux/const.h
LIB_H += ../../include/linux/hash.h
LIB_H += ../../include/linux/stringify.h
LIB_H += util/include/linux/bitmap.h
LIB_H += util/include/linux/bitops.h
LIB_H += util/include/linux/compiler.h
LIB_H += util/include/linux/const.h
LIB_H += util/include/linux/ctype.h
LIB_H += util/include/linux/kernel.h
LIB_H += util/include/linux/list.h
LIB_H += util/include/linux/export.h
LIB_H += util/include/linux/magic.h
LIB_H += util/include/linux/poison.h
LIB_H += util/include/linux/prefetch.h
LIB_H += util/include/linux/rbtree.h
LIB_H += util/include/linux/rbtree_augmented.h
LIB_H += util/include/linux/string.h
LIB_H += util/include/linux/types.h
LIB_H += util/include/linux/linkage.h
LIB_H += util/include/asm/asm-offsets.h
LIB_H += util/include/asm/bug.h
LIB_H += util/include/asm/byteorder.h
LIB_H += util/include/asm/hweight.h
LIB_H += util/include/asm/swab.h
LIB_H += util/include/asm/system.h
LIB_H += util/include/asm/uaccess.h
LIB_H += util/include/dwarf-regs.h
LIB_H += util/include/asm/dwarf2.h
LIB_H += util/include/asm/cpufeature.h
LIB_H += util/include/asm/unistd_32.h
LIB_H += util/include/asm/unistd_64.h
LIB_H += perf.h
LIB_H += util/annotate.h
LIB_H += util/cache.h
LIB_H += util/callchain.h
LIB_H += util/build-id.h
LIB_H += util/debug.h
LIB_H += util/sysfs.h
LIB_H += util/pmu.h
LIB_H += util/event.h
LIB_H += util/evsel.h
LIB_H += util/evlist.h
LIB_H += util/exec_cmd.h
LIB_H += util/types.h
LIB_H += util/levenshtein.h
LIB_H += util/machine.h
LIB_H += util/map.h
LIB_H += util/parse-options.h
LIB_H += util/parse-events.h
LIB_H += util/quote.h
LIB_H += util/util.h
LIB_H += util/xyarray.h
LIB_H += util/header.h
LIB_H += util/help.h
LIB_H += util/session.h
LIB_H += util/strbuf.h
LIB_H += util/strlist.h
LIB_H += util/strfilter.h
LIB_H += util/svghelper.h
LIB_H += util/tool.h
LIB_H += util/run-command.h
LIB_H += util/sigchain.h
LIB_H += util/dso.h
LIB_H += util/symbol.h
LIB_H += util/color.h
LIB_H += util/values.h
LIB_H += util/sort.h
LIB_H += util/hist.h
LIB_H += util/thread.h
LIB_H += util/thread_map.h
LIB_H += util/trace-event.h
LIB_H += util/probe-finder.h
LIB_H += util/dwarf-aux.h
LIB_H += util/probe-event.h
LIB_H += util/pstack.h
LIB_H += util/cpumap.h
LIB_H += util/top.h
LIB_H += $(ARCH_INCLUDE)
LIB_H += util/cgroup.h
LIB_H += $(TRACE_EVENT_DIR)event-parse.h
LIB_H += util/target.h
LIB_H += util/rblist.h
LIB_H += util/intlist.h
LIB_H += util/perf_regs.h
LIB_H += util/unwind.h
LIB_H += util/vdso.h
LIB_H += ui/helpline.h
LIB_H += ui/progress.h
LIB_H += ui/util.h
LIB_H += ui/ui.h

LIB_OBJS += $(OUTPUT)util/abspath.o
LIB_OBJS += $(OUTPUT)util/alias.o
LIB_OBJS += $(OUTPUT)util/annotate.o
LIB_OBJS += $(OUTPUT)util/build-id.o
LIB_OBJS += $(OUTPUT)util/config.o
LIB_OBJS += $(OUTPUT)util/ctype.o
LIB_OBJS += $(OUTPUT)util/sysfs.o
LIB_OBJS += $(OUTPUT)util/pmu.o
LIB_OBJS += $(OUTPUT)util/environment.o
LIB_OBJS += $(OUTPUT)util/event.o
LIB_OBJS += $(OUTPUT)util/evlist.o
LIB_OBJS += $(OUTPUT)util/evsel.o
LIB_OBJS += $(OUTPUT)util/exec_cmd.o
LIB_OBJS += $(OUTPUT)util/help.o
LIB_OBJS += $(OUTPUT)util/levenshtein.o
LIB_OBJS += $(OUTPUT)util/parse-options.o
LIB_OBJS += $(OUTPUT)util/parse-events.o
LIB_OBJS += $(OUTPUT)util/path.o
LIB_OBJS += $(OUTPUT)util/rbtree.o
LIB_OBJS += $(OUTPUT)util/bitmap.o
LIB_OBJS += $(OUTPUT)util/hweight.o
LIB_OBJS += $(OUTPUT)util/run-command.o
LIB_OBJS += $(OUTPUT)util/quote.o
LIB_OBJS += $(OUTPUT)util/strbuf.o
LIB_OBJS += $(OUTPUT)util/string.o
LIB_OBJS += $(OUTPUT)util/strlist.o
LIB_OBJS += $(OUTPUT)util/strfilter.o
LIB_OBJS += $(OUTPUT)util/top.o
LIB_OBJS += $(OUTPUT)util/usage.o
LIB_OBJS += $(OUTPUT)util/wrapper.o
LIB_OBJS += $(OUTPUT)util/sigchain.o
LIB_OBJS += $(OUTPUT)util/dso.o
LIB_OBJS += $(OUTPUT)util/symbol.o
LIB_OBJS += $(OUTPUT)util/symbol-elf.o
LIB_OBJS += $(OUTPUT)util/color.o
LIB_OBJS += $(OUTPUT)util/pager.o
LIB_OBJS += $(OUTPUT)util/header.o
LIB_OBJS += $(OUTPUT)util/callchain.o
LIB_OBJS += $(OUTPUT)util/values.o
LIB_OBJS += $(OUTPUT)util/debug.o
LIB_OBJS += $(OUTPUT)util/machine.o
LIB_OBJS += $(OUTPUT)util/map.o
LIB_OBJS += $(OUTPUT)util/pstack.o
LIB_OBJS += $(OUTPUT)util/session.o
LIB_OBJS += $(OUTPUT)util/thread.o
LIB_OBJS += $(OUTPUT)util/thread_map.o
LIB_OBJS += $(OUTPUT)util/trace-event-parse.o
LIB_OBJS += $(OUTPUT)util/parse-events-flex.o
LIB_OBJS += $(OUTPUT)util/parse-events-bison.o
LIB_OBJS += $(OUTPUT)util/pmu-flex.o
LIB_OBJS += $(OUTPUT)util/pmu-bison.o
LIB_OBJS += $(OUTPUT)util/trace-event-read.o
LIB_OBJS += $(OUTPUT)util/trace-event-info.o
LIB_OBJS += $(OUTPUT)util/trace-event-scripting.o
LIB_OBJS += $(OUTPUT)util/svghelper.o
LIB_OBJS += $(OUTPUT)util/sort.o
LIB_OBJS += $(OUTPUT)util/hist.o
LIB_OBJS += $(OUTPUT)util/probe-event.o
LIB_OBJS += $(OUTPUT)util/util.o
LIB_OBJS += $(OUTPUT)util/xyarray.o
LIB_OBJS += $(OUTPUT)util/cpumap.o
LIB_OBJS += $(OUTPUT)util/cgroup.o
LIB_OBJS += $(OUTPUT)util/target.o
LIB_OBJS += $(OUTPUT)util/rblist.o
LIB_OBJS += $(OUTPUT)util/intlist.o
LIB_OBJS += $(OUTPUT)util/vdso.o
LIB_OBJS += $(OUTPUT)util/stat.o

LIB_OBJS += $(OUTPUT)ui/setup.o
LIB_OBJS += $(OUTPUT)ui/helpline.o
LIB_OBJS += $(OUTPUT)ui/progress.o
LIB_OBJS += $(OUTPUT)ui/util.o
LIB_OBJS += $(OUTPUT)ui/hist.o
LIB_OBJS += $(OUTPUT)ui/stdio/hist.o

LIB_OBJS += $(OUTPUT)arch/common.o

LIB_OBJS += $(OUTPUT)tests/parse-events.o
LIB_OBJS += $(OUTPUT)tests/dso-data.o
LIB_OBJS += $(OUTPUT)tests/attr.o
LIB_OBJS += $(OUTPUT)tests/vmlinux-kallsyms.o
LIB_OBJS += $(OUTPUT)tests/open-syscall.o
LIB_OBJS += $(OUTPUT)tests/open-syscall-all-cpus.o
LIB_OBJS += $(OUTPUT)tests/open-syscall-tp-fields.o
LIB_OBJS += $(OUTPUT)tests/mmap-basic.o
LIB_OBJS += $(OUTPUT)tests/perf-record.o
LIB_OBJS += $(OUTPUT)tests/rdpmc.o
LIB_OBJS += $(OUTPUT)tests/evsel-roundtrip-name.o
LIB_OBJS += $(OUTPUT)tests/evsel-tp-sched.o
LIB_OBJS += $(OUTPUT)tests/pmu.o
LIB_OBJS += $(OUTPUT)tests/hists_link.o
LIB_OBJS += $(OUTPUT)tests/python-use.o
LIB_OBJS += $(OUTPUT)tests/bp_signal.o
LIB_OBJS += $(OUTPUT)tests/bp_signal_overflow.o
LIB_OBJS += $(OUTPUT)tests/task-exit.o
LIB_OBJS += $(OUTPUT)tests/sw-clock.o

BUILTIN_OBJS += $(OUTPUT)builtin-annotate.o
BUILTIN_OBJS += $(OUTPUT)builtin-bench.o
# Benchmark modules
BUILTIN_OBJS += $(OUTPUT)bench/sched-messaging.o
BUILTIN_OBJS += $(OUTPUT)bench/sched-pipe.o
ifeq ($(RAW_ARCH),x86_64)
BUILTIN_OBJS += $(OUTPUT)bench/mem-memcpy-x86-64-asm.o
BUILTIN_OBJS += $(OUTPUT)bench/mem-memset-x86-64-asm.o
endif
BUILTIN_OBJS += $(OUTPUT)bench/mem-memcpy.o
BUILTIN_OBJS += $(OUTPUT)bench/mem-memset.o

BUILTIN_OBJS += $(OUTPUT)builtin-diff.o
BUILTIN_OBJS += $(OUTPUT)builtin-evlist.o
BUILTIN_OBJS += $(OUTPUT)builtin-help.o
BUILTIN_OBJS += $(OUTPUT)builtin-sched.o
BUILTIN_OBJS += $(OUTPUT)builtin-buildid-list.o
BUILTIN_OBJS += $(OUTPUT)builtin-buildid-cache.o
BUILTIN_OBJS += $(OUTPUT)builtin-list.o
BUILTIN_OBJS += $(OUTPUT)builtin-record.o
BUILTIN_OBJS += $(OUTPUT)builtin-report.o
BUILTIN_OBJS += $(OUTPUT)builtin-stat.o
BUILTIN_OBJS += $(OUTPUT)builtin-periodic.o
BUILTIN_OBJS += $(OUTPUT)builtin-timechart.o
BUILTIN_OBJS += $(OUTPUT)builtin-top.o
BUILTIN_OBJS += $(OUTPUT)builtin-script.o
BUILTIN_OBJS += $(OUTPUT)builtin-probe.o
BUILTIN_OBJS += $(OUTPUT)builtin-kmem.o
BUILTIN_OBJS += $(OUTPUT)builtin-lock.o
BUILTIN_OBJS += $(OUTPUT)builtin-kvm.o
BUILTIN_OBJS += $(OUTPUT)builtin-inject.o
BUILTIN_OBJS += $(OUTPUT)tests/builtin-test.o
BUILTIN_OBJS += $(OUTPUT)builtin-mem.o

PERFLIBS = $(LIB_FILE) $(LIBLK) $(LIBTRACEEVENT)
>>>>>>> p9x

#
# Do a parallel build with multiple jobs, based on the number of CPUs online
# in this system: 'make -j8' on a 8-CPU system, etc.
#
# (To override it, run 'make JOBS=1' and similar.)
#
ifeq ($(JOBS),)
  JOBS := $(shell grep -c ^processor /proc/cpuinfo 2>/dev/null)
  ifeq ($(JOBS),)
    JOBS := 1
  endif
endif

#
# Only pass canonical directory names as the output directory:
#
ifneq ($(O),)
  FULL_O := $(shell readlink -f $(O) || echo $(O))
endif

#
# Only accept the 'DEBUG' variable from the command line:
#
ifeq ("$(origin DEBUG)", "command line")
  ifeq ($(DEBUG),)
    override DEBUG = 0
  else
    SET_DEBUG = "DEBUG=$(DEBUG)"
  endif
else
  override DEBUG = 0
endif

define print_msg
  @printf '  BUILD:   Doing '\''make \033[33m-j'$(JOBS)'\033[m'\'' parallel build\n'
endef

define make
  @$(MAKE) -f Makefile.perf --no-print-directory -j$(JOBS) O=$(FULL_O) $(SET_DEBUG) $@
endef

#
# Needed if no target specified:
# (Except for tags and TAGS targets. The reason is that the
# Makefile does not treat tags/TAGS as targets but as files
# and thus won't rebuilt them once they are in place.)
#
all tags TAGS:
	$(print_msg)
	$(make)

#
# Explicitly disable parallelism for the clean target.
#
clean:
	$(make) -j1

#
# The build-test target is not really parallel, don't print the jobs info:
#
build-test:
	@$(MAKE) -f tests/make --no-print-directory

#
# All other targets get passed through:
#
%:
	$(print_msg)
	$(make)

.PHONY: tags TAGS
