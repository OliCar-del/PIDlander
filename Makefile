# Minimal build for the PID lander (Linux / macOS).
# Windows: use MSYS2 (pacman -S mingw-w64-ucrt-x86_64-raylib) or raylib's installer.

CC     ?= gcc
CFLAGS ?= -Wall -Wextra -std=c99 -O2

# Prefer pkg-config when available (e.g. Homebrew raylib on macOS);
# otherwise fall back to the standard Linux link line for a static raylib.
RAYLIB_FLAGS := $(shell pkg-config --cflags --libs raylib 2>/dev/null)
ifeq ($(RAYLIB_FLAGS),)
RAYLIB_FLAGS := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
endif

SRC := main.c pid.c ui.c util.c sim.c chart.c seq.c plots.c autotune.c
HDR := config.h pid.h ui.h util.h sim.h chart.h seq.h plots.h autotune.h

lander: $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(RAYLIB_FLAGS)
ifeq ($(OS),Windows_NT)
	# MSYS2's raylib is a DLL; keep it (and its glfw3 dependency) next to
	# the exe so lander.exe runs outside the MSYS2 shell (e.g. double-click).
	cp -n /ucrt64/bin/libraylib.dll /ucrt64/bin/glfw3.dll . 2>/dev/null || true
endif

run: lander
	./lander

# Headless tests: PID closed-loop metrics + chart ring-buffer integrity.
test: test_pid.c pid.c pid.h test_chart.c chart.c sim.c util.c
	$(CC) $(CFLAGS) test_pid.c pid.c -o test_pid -lm
	./test_pid
	$(CC) $(CFLAGS) test_chart.c chart.c sim.c util.c -o test_chart $(RAYLIB_FLAGS)
	./test_chart

clean:
	rm -f lander lander.exe test_pid test_pid.exe test_chart test_chart.exe

.PHONY: run test clean
