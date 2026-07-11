// seq.h — target sequence editor: numbered draggable points defining a
// stepped setpoint-vs-time profile, played back on sim time.

#ifndef SEQ_H
#define SEQ_H

#include <stdbool.h>

#include "config.h"

typedef struct Seq {
    int   n;              // active points
    float T;              // timeframe [s]
    float t[SEQ_MAX];     // point times [s], kept ordered
    float y[SEQ_MAX];     // point heights [m]
    bool  playing;
    float t_play;         // playback clock [s], advances with sim time
    float base;           // setpoint captured when PLAY was pressed
    int   drag;           // point index being dragged, -1 = none
} Seq;

// Stepped profile: hold base until point 1, then each height until the next.
float seq_eval(const Seq *q, float t);

// Draw the editor panel and handle its widgets and point dragging.
// live_setpoint is the current target (baseline of the profile when idle).
void seq_panel(Seq *q, float live_setpoint);

#endif // SEQ_H
