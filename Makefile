# ============================================================================
#  Makefile ??? Linux and macOS (gcc or clang). Windows users: use build.bat.
#
#  There are NO external dependencies. Only the C standard library, plus -lm
#  for the maths functions on Linux (a no-op on macOS, where libm is folded
#  into libSystem, but harmless to pass).
#
#      make            build everything into out/
#      make test       build and run the unit tests
#      make asan       rebuild everything under AddressSanitizer + UBSan
#      make run-link   run the full link simulation
#      make clean
#
#  Override the compiler or flags as usual:
#      make CC=clang
#      make CFLAGS="-O0 -g"
# ============================================================================

CC       ?= cc
CSTD     := -std=c17
CFLAGS   ?= -O2
WARN     ?= -Wall -Wextra -Wpedantic
STRICT   := -Wall -Wextra -Wpedantic -Wshadow -Wcast-qual \
            -Wstrict-prototypes -Wmissing-prototypes -Wconversion -Wsign-conversion
INC      := -Iinclude
LDLIBS   := -lm
OUT      := out

SRC      := $(sort $(wildcard src/*.c))
APP_SRC  := $(sort $(wildcard apps/*.c))
APPS     := $(notdir $(basename $(APP_SRC)))
TARGETS  := $(addprefix $(OUT)/,$(APPS) test_all)

.PHONY: all test asan strict clean run-link run-eye help

all: $(TARGETS)

$(OUT):
	@mkdir -p $(OUT)

# unit tests live in tests/, everything else in apps/
$(OUT)/test_all: tests/test_all.c $(SRC) | $(OUT)
	$(CC) $(CSTD) $(CFLAGS) $(WARN) $(INC) $^ -o $@ $(LDLIBS)

$(OUT)/%: apps/%.c $(SRC) | $(OUT)
	$(CC) $(CSTD) $(CFLAGS) $(WARN) $(INC) $^ -o $@ $(LDLIBS)

test: $(OUT)/test_all
	@./$(OUT)/test_all

# Sanitisers catch what the warnings do not: out-of-bounds, use-after-free,
# signed overflow, misaligned access. Worth running before believing a result.
asan:
	@$(MAKE) --no-print-directory clean
	@$(MAKE) --no-print-directory CFLAGS="-O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined" all
	@./$(OUT)/test_all

# Opt-in pedantic pass. Not the default: -Wconversion on a DSP codebase that
# moves between int, size_t and double is mostly noise, and a wall of warnings
# on first build is worse than none.
strict:
	@$(MAKE) --no-print-directory clean
	@$(MAKE) --no-print-directory WARN="$(STRICT)" all

run-link: $(OUT)/link_sim
	@./$(OUT)/link_sim 20 120

run-eye: $(OUT)/link_sim
	@./$(OUT)/link_sim 20 120 && echo "wrote eye.pgm"

clean:
	@rm -rf $(OUT) *.pgm

help:
	@echo "targets: all test asan run-link run-eye clean"
	@echo "apps:    $(APPS) test_all"

