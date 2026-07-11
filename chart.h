// chart.h — 20 s scrolling history: altitude/setpoint strip over a separate
// thrust/wind force strip (they deliberately do NOT share an axis, so 0 N of
// thrust can never be mistaken for 0 m of altitude). Newest sample at the
// right edge.

#ifndef CHART_H
#define CHART_H

#include "config.h"

// per-sample event flags (CF_GROUND is set by chart_push itself)
enum { CF_GROUND = 1, CF_RESET = 2 };

typedef struct Chart {
    float y[CHART_N];    // altitude [m]
    float sp[CHART_N];   // setpoint [m]
    float u[CHART_N];    // applied thrust [N]
    float w[CHART_N];    // wind force [N]
    float v[CHART_N];    // velocity [m/s] (used by the phase portrait)
    unsigned char flag[CHART_N];
    int head;            // next write slot
    int count;           // valid samples so far (saturates at CHART_N)
    int div;             // physics-step divider for the sample rate
} Chart;

void chart_push(Chart *c, float y, float sp, float u, float w, float v,
                unsigned char flags);
void chart_draw(const Chart *c, float u_max);

#endif // CHART_H
