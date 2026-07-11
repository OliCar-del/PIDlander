// util.h — small shared helpers.

#ifndef UTIL_H
#define UTIL_H

float clampf(float v, float lo, float hi);

// Uniform random float in [lo, hi] (raylib's RNG).
float frand(float lo, float hi);

// Standard normal via Box-Muller.
float gauss(void);

#endif // UTIL_H
