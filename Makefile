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

SRC := main.c pid.c ui.c

lander: $(SRC) pid.h ui.h
	$(CC) $(CFLAGS) $(SRC) -o $@ $(RAYLIB_FLAGS)
ifeq ($(OS),Windows_NT)
	# MSYS2's raylib is a DLL; keep it (and its glfw3 dependency) next to
	# the exe so lander.exe runs outside the MSYS2 shell (e.g. double-click).
	cp -n /ucrt64/bin/libraylib.dll /ucrt64/bin/glfw3.dll . 2>/dev/null || true
endif

run: lander
	./lander

# Headless closed-loop tests: no raylib needed, prints step/gust metrics.
test: test_pid.c pid.c pid.h
	$(CC) $(CFLAGS) test_pid.c pid.c -o test_pid -lm
	./test_pid

clean:
	rm -f lander lander.exe test_pid test_pid.exe

.PHONY: run test clean
