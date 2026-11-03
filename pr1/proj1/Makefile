# Makefile - CSE 4360/5364 Project 1 (c4ev3 / EV3-API)
#
#   make            build proj1.elf + proj1.rbf  (cross-compiled for the EV3)
#   make test       build + run the planner on the laptop (ASCII map)
#   make upload     copy proj1.elf + proj1.rbf to the brick over USB (ev3duder)
#   make run        upload and start the program on the brick
#   make clean
#
# Variables you may need to override (or edit here):
#   CROSS_COMPILE   compiler prefix.  Default: tools/arm-ev3-  (wrapper around
#                   "zig cc", see README "Toolchain").  Alternatives:
#                   arm-c4ev3-linux-uclibceabi-  (official c4ev3 toolchain)
#                   arm-none-linux-gnueabi-      (CodeSourcery 2009q1)
#                   !! NOT arm-linux-gnueabi- from Mint/Ubuntu apt: its glibc
#                   !! needs kernel >= 3.2, the EV3 runs 2.6.33 -> "kernel too old"
#   APIDIR          path to a checkout of https://github.com/c4ev3/EV3-API
#   EV3DUDER        path/name of the ev3duder binary
#   REMDIR          directory on the brick (default: SD card, "myapps" folder,
#                   which is what the LEGO menu shows under SD_Card/myapps)
#
# Example:  make CROSS_COMPILE=arm-c4ev3-linux-uclibceabi- APIDIR=$HOME/ev3/EV3-API

CROSS_COMPILE ?= $(CURDIR)/tools/arm-ev3-
APIDIR        ?= ../EV3-API
EV3DUDER      ?= ev3duder
REMDIR        ?= /media/card/myapps

CC      = $(CROSS_COMPILE)gcc
HOSTCC  = gcc
APP     = proj1
ELF     = $(APP).elf
RBF     = $(APP).rbf
SRCS    = main.c robot.c planner.c params.c pid.c
OBJS    = $(SRCS:.c=.o)
APILIB  = $(APIDIR)/libev3api.a

CFLAGS  += -std=c99 -Os -Wall -Wextra -Wno-unused-parameter -Wno-unused-function
CFLAGS  += -fno-strict-aliasing -fwrapv -D_GNU_SOURCE=1
CFLAGS  += -I$(APIDIR)/include -I$(APIDIR)/API -I$(APIDIR)
LDFLAGS += -static
LDLIBS  += -L$(APIDIR) -lev3api -lm -lpthread

all: $(ELF) $(RBF)

$(ELF): $(OBJS) $(APILIB)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@ $(LDLIBS)
	@echo "built $@"

%.o: %.c config.h courses/course.h planner.h robot.h params.h pid.h
	$(CC) $(CFLAGS) -c $< -o $@

# Build the EV3-API static library with the same cross compiler
$(APILIB):
	$(MAKE) -C $(APIDIR) CROSS_COMPILE=$(CROSS_COMPILE) libev3api.a

# The .rbf is a tiny LEGO "program" that just execs our ELF.  We generate
# it with the bundled mkrbf.py (no ev3duder needed).  The programs open
# their own log file (proj1.log etc.) next to the ELF; no shell redirect in
# the .rbf, because a failed redirect (read-only card) would silently
# prevent the program from starting at all.
$(RBF): $(ELF) mkrbf.py
	python3 mkrbf.py "$(REMDIR)/$(ELF)" $@

# ---- laptop dry run of the real main.c (no robot) ------------------------
#   ./proj1_host                     plan + dry-run the compiled-in course
#   ./proj1_host courses/snake3.txt  any course file;  add "worst" / "dist"
host: proj1_host
proj1_host: main.c planner.c params.c host/robot_host.c host/ev3_stub.h *.h courses/course.h
	$(HOSTCC) -std=c99 -O2 -Wall -DHOST_BUILD -I. main.c planner.c params.c host/robot_host.c -o $@ -lm

# ---- laptop-side planner test -------------------------------------------
test_planner: test_planner.c planner.c params.c planner.h params.h config.h courses/course.h
	$(HOSTCC) -std=c99 -O2 -Wall -DHOST_BUILD -o $@ test_planner.c planner.c params.c -lm

test: test_planner
	./test_planner

# preview every sample course, best and worst path
courses: test_planner
	@for f in courses/*.txt; do echo; echo "######## $$f"; ./test_planner $$f summary; done

# ---- brick interaction (USB) -------------------------------------------
upload: $(ELF) $(RBF)
	-$(EV3DUDER) mkdir "$(REMDIR)"
	$(EV3DUDER) up $(ELF) "$(REMDIR)/$(ELF)"
	$(EV3DUDER) up $(RBF) "$(REMDIR)/$(RBF)"

run: upload
	$(EV3DUDER) run "$(REMDIR)/$(RBF)"

log:
	$(EV3DUDER) cat "$(REMDIR)/$(APP).log"

clean:
	rm -f $(OBJS) $(ELF) $(RBF) test_planner proj1_host move1s.elf move1s.rbf trim.elf trim.rbf diag.elf diag.rbf

cleanapi:
	$(MAKE) -C $(APIDIR) clean

.PHONY: all host test courses upload run log clean cleanapi

# ---- smoke test: drive forward for 1 s (tests/move1s.c) -----------------
move1s: move1s.elf move1s.rbf trim.elf trim.rbf diag.elf diag.rbf

move1s.elf: tests/move1s.c config.h $(APILIB)
	$(CC) $(CFLAGS) $(LDFLAGS) tests/move1s.c -o $@ $(LDLIBS)

move1s.rbf: move1s.elf mkrbf.py
	python3 mkrbf.py "$(REMDIR)/move1s.elf" $@

.PHONY: move1s

# ---- trim test: measure left/right pull, suggest MOTOR_TRIM (tests/trim.c)
trim: trim.elf trim.rbf diag.elf diag.rbf

trim.elf: tests/trim.c config.h $(APILIB)
	$(CC) $(CFLAGS) $(LDFLAGS) tests/trim.c -o $@ $(LDLIBS)

trim.rbf: trim.elf mkrbf.py
	python3 mkrbf.py "$(REMDIR)/trim.elf" $@

.PHONY: trim

# ---- diagnostics suite (tests/diag.c + robot.c) --------------------------
diag: diag.elf diag.rbf

diag.elf: tests/diag.c robot.c params.c pid.c robot.h params.h pid.h config.h $(APILIB)
	$(CC) $(CFLAGS) -I. $(LDFLAGS) tests/diag.c robot.c params.c pid.c -o $@ $(LDLIBS)

diag.rbf: diag.elf mkrbf.py
	python3 mkrbf.py "$(REMDIR)/diag.elf" $@

.PHONY: diag
