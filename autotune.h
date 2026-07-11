// autotune.h — relay (Astrom-Hagglund) experiment: force a limit cycle with
// bang-bang thrust, measure its amplitude and period, derive the ultimate
// gain Ku and period Tu for tuning rules.

#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <stdbool.h>

typedef enum { AT_RUNNING, AT_DONE, AT_FAILED } AtStatus;

typedef struct Autotune {
    bool  running;
    float d;            // relay amplitude [N]
    float h;            // switching hysteresis [m]
    int   sign;         // current relay output sign
    int   ups;          // up-switch count (first two = transient, skipped)
    float last_up_t;    // sim time of the previous counted up-switch
    float periods[4];
    int   np;
    float ymin, ymax;   // oscillation envelope during the measuring window
    float t_start;
    float ku, tu;       // measured ultimate gain/period (0 until measured)
    char  result[96];
    float result_t;     // sim time the result banner was set
} Autotune;

// Start the experiment; keeps any previously measured Ku/Tu until remeasured.
void autotune_begin(Autotune *at, float d, float h, float t_now);

// Stop without a result and show msg as the banner.
void autotune_cancel(Autotune *at, const char *msg, float t_now);

// One 120 Hz step. e = setpoint - measured altitude; y_true = true altitude.
// Writes the relay command (relative to feedforward) to *u_rel. Returns
// AT_DONE when a fresh Ku/Tu is available, AT_FAILED on abort.
AtStatus autotune_step(Autotune *at, float e, float y_true, float t_now,
                       float *u_rel);

#endif // AUTOTUNE_H
