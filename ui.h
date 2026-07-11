// ui.h — minimal immediate-mode widgets for raylib.
//
// "Immediate mode" means there is no widget tree: each widget is a single
// function call made every frame that both DRAWS the widget and HANDLES its
// mouse interaction, returning the (possibly updated) value. State that must
// survive between frames (which slider the mouse has grabbed) is kept in one
// module-private variable, keyed by the caller-supplied id.
//
// Rules for callers:
//   - call each widget exactly once per frame, between BeginDrawing() and
//     EndDrawing()
//   - give every slider a unique id so a drag stays attached to the slider
//     it started on, even when the pointer moves off it

#ifndef UI_H
#define UI_H

#include "raylib.h"
#include <stdbool.h>

// Horizontal slider. Draws a label (top left), the formatted current value
// (top right), and a track with a draggable knob underneath. Occupies a
// region roughly w wide by 34 px tall with its top-left corner at (x, y).
//   id     unique per slider, any integer
//   value  current value; returned unchanged unless the user drags
//   lo/hi  value range mapped onto the track ends
//   fmt    printf format for the value readout, e.g. "%.2f"
// Returns the new value.
float ui_slider(int id, int x, int y, int w, const char *label,
                float value, float lo, float hi, const char *fmt);

// Momentary button. Draws highlighted when it represents the active choice
// (e.g. the selected mode in a group). Returns true only on the frame the
// user clicks it.
bool ui_button(Rectangle r, const char *label, bool highlighted);

// True while any slider is being dragged. Other mouse handlers (e.g. the
// click-to-set-target flight view) should yield while this is true.
bool ui_dragging(void);

#endif // UI_H
