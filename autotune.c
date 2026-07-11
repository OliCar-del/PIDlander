#include "autotune.h"

#include <stdio.h>

#include "config.h"

void autotune_begin(Autotune *at, float d, float h, float t_now)
{
    float ku = at->ku, tu = at->tu;     // rule presets stay unlocked
    *at = (Autotune){ 0 };
    at->ku = ku;
    at->tu = tu;
    at->running = true;
    at->d = d;
    at->h = h;
    at->sign = +1;
    at->t_start = t_now;
}

void autotune_cancel(Autotune *at, const char *msg, float t_now)
{
    at->running = false;
    snprintf(at->result, sizeof at->result, "%s", msg);
    at->result_t = t_now;
}

AtStatus autotune_step(Autotune *at, float e, float y_true, float t_now,
                       float *u_rel)
{
    // relay with hysteresis: flip only after the error clearly crosses over
    if (at->sign > 0 && e < -at->h) {
        at->sign = -1;
    } else if (at->sign < 0 && e > at->h) {
        at->sign = +1;
        at->ups++;
        if (at->ups == 3) {                 // transient over: start measuring
            at->ymin = at->ymax = y_true;
            at->last_up_t = t_now;
        } else if (at->ups > 3) {
            at->periods[at->np++] = t_now - at->last_up_t;
            at->last_up_t = t_now;
        }
    }
    if (at->ups >= 3) {
        if (y_true < at->ymin) at->ymin = y_true;
        if (y_true > at->ymax) at->ymax = y_true;
    }
    *u_rel = (float)at->sign * at->d;

    if (at->np >= 3) {                      // enough cycles: compute Ku/Tu
        float tu = (at->periods[0] + at->periods[1] + at->periods[2]) / 3.0f;
        float a = (at->ymax - at->ymin) / 2.0f;
        at->running = false;
        at->result_t = t_now;
        if (a > 0.01f && tu > 0.05f) {
            at->ku = 4.0f * at->d / (PI * a);
            at->tu = tu;
            snprintf(at->result, sizeof at->result,
                     "AUTOTUNE: Ku=%.1f Tu=%.2fs -> Ziegler-Nichols applied "
                     "(more rules unlocked)", at->ku, at->tu);
            return AT_DONE;
        }
        snprintf(at->result, sizeof at->result,
                 "AUTOTUNE failed: oscillation too small");
        return AT_FAILED;
    }
    if (t_now - at->t_start > 30.0f) {
        at->running = false;
        at->result_t = t_now;
        snprintf(at->result, sizeof at->result,
                 "AUTOTUNE failed: no steady oscillation in 30 s");
        return AT_FAILED;
    }
    return AT_RUNNING;
}
