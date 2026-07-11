#include "util.h"

#include <math.h>

#include "raylib.h"
#include "config.h"

float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float frand(float lo, float hi)
{
    return lo + (hi - lo) * (float)GetRandomValue(0, 10000) / 10000.0f;
}

float gauss(void)
{
    float u1 = frand(1e-6f, 1.0f), u2 = frand(0.0f, 1.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}
