# ---------------------------------------------------------------------------
# ADAS closed-loop simulator
#
#   make                  build (optimized)             -> build/release/adas_sim
#   make run              build + run the scenario      -> out/<scenario>_seed<N>.csv
#   make debug            -O0 -g + ASan/UBSan build     -> build/debug/adas_sim
#   make debug-test       unit/integration tests under ASan/UBSan
#   make plot             run + plot (python3 + matplotlib) -> out/<log>.pdf
#   make test             build + run unit/integration tests
#   make format           reformat all sources with clang-format
#   make format-check     fail if any source is not clang-format clean
#   make lint             run clang-tidy (uses compile_commands.json)
#   make compile_commands.json   compilation database for clangd / clang-tidy
#   make check            format-check + warnings-as-errors build + test + lint
#   make clean            remove build/ (out/ is kept)
#
#   make WERROR=1 ...     treat compiler warnings as errors
#   make CC=gcc ...       use another compiler (default: clang)
#
# New .c files anywhere under src/ or tests/ are picked up automatically.
# ---------------------------------------------------------------------------

# Make predefines CC=cc, so `CC ?= clang` would never apply: replace only
# that built-in default (`make CC=gcc` or an exported CC still wins).
ifeq ($(origin CC),default)
CC       := clang
endif
BUILD    ?= build/release
WERROR   ?= 0
TARGET   := $(BUILD)/adas_sim
TEST_BIN := $(BUILD)/run_tests

SRC_DIR  := src
TEST_DIR := tests
SRCS     := $(shell find $(SRC_DIR) -name '*.c' | sort)
LIB_SRCS := $(filter-out $(SRC_DIR)/app/main.c,$(SRCS))
TEST_SRCS := $(shell find $(TEST_DIR) -name '*.c' | sort)
HDRS     := $(shell find include config $(TEST_DIR) -name '*.h' | sort)

OBJS      := $(SRCS:%.c=$(BUILD)/%.o)
LIB_OBJS  := $(LIB_SRCS:%.c=$(BUILD)/%.o)
TEST_OBJS := $(TEST_SRCS:%.c=$(BUILD)/%.o)
DEPS      := $(OBJS:.o=.d) $(TEST_OBJS:.o=.d)

CSTD     := -std=c11
WARNINGS := -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes \
            -Wmissing-prototypes -Wconversion -Wdouble-promotion -Wcast-qual \
            -Wformat=2
ifeq ($(WERROR),1)
WARNINGS += -Werror
endif
OPTFLAGS ?= -O2
CPPFLAGS := -Iinclude -Iconfig -D_POSIX_C_SOURCE=200809L -MMD -MP
CFLAGS   := $(CSTD) $(WARNINGS) $(OPTFLAGS)
LDLIBS   := -lm

FORMAT_FILES := $(SRCS) $(TEST_SRCS) $(HDRS)

.PHONY: all run debug debug-test plot test format format-check lint check clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(TEST_BIN): $(LIB_OBJS) $(TEST_OBJS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

# ASan + UBSan; any UBSan report aborts, so it fails the run.
SANITIZE := -fsanitize=address,undefined -fno-sanitize-recover=all
DEBUG_VARS := BUILD=build/debug OPTFLAGS="-O0 -g3 $(SANITIZE)" \
              LDFLAGS="$(SANITIZE)"

debug:
	$(MAKE) $(DEBUG_VARS)

debug-test:
	$(MAKE) $(DEBUG_VARS) test

plot: run
	python3 tools/plot_log.py

test: $(TEST_BIN)
	./$(TEST_BIN)

format:
	clang-format -i $(FORMAT_FILES)

format-check:
	clang-format --dry-run -Werror $(FORMAT_FILES)

# Compilation database for clangd / clang-tidy (paths are relative to this
# directory, so the file is only valid for this checkout).
JSON_FLAGS := $(foreach f,$(filter-out -MMD -MP,$(CPPFLAGS) $(CFLAGS)),"$(f)",)

compile_commands.json: Makefile
	@echo "generating $@"
	@{ echo '['; sep=''; \
	   for f in $(SRCS) $(TEST_SRCS); do \
	     printf '%s  {"directory": "%s", "file": "%s", "arguments": ["$(CC)", $(JSON_FLAGS) "-c", "%s"]}' \
	       "$$sep" "$(CURDIR)" "$$f" "$$f"; sep=$$',\n'; \
	   done; \
	   for f in $(HDRS); do \
	     printf '%s  {"directory": "%s", "file": "%s", "arguments": ["$(CC)", $(JSON_FLAGS) "-xc-header", "%s"]}' \
	       "$$sep" "$(CURDIR)" "$$f" "$$f"; sep=$$',\n'; \
	   done; \
	   printf '\n]\n'; } > $@

lint: compile_commands.json
	clang-tidy -p . --quiet $(SRCS) $(TEST_SRCS)

check: format-check
	$(MAKE) BUILD=build/check WERROR=1 test
	$(MAKE) lint

clean:
	rm -rf build

-include $(DEPS)
