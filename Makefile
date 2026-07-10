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

lander: main.c
	$(CC) $(CFLAGS) $< -o $@ $(RAYLIB_FLAGS)

run: lander
	./lander

clean:
	rm -f lander

.PHONY: run clean
