// ui.h — minimal immediate-mode widgets for raylib (sliders + buttons).
// Call once per frame per widget, inside BeginDrawing/EndDrawing; each
// slider needs a unique id so drags stay captured by the widget they
// started on.

#ifndef UI_H
#define UI_H

#include "raylib.h"
#include <stdbool.h>

// Horizontal slider with a label (left) and formatted value (right).
// Returns the possibly-updated value. Occupies roughly w x 34 px at (x,y).
float ui_slider(int id, int x, int y, int w, const char *label,
                float value, float lo, float hi, const char *fmt);

// Clickable button; draw highlighted when it represents the active choice.
// Returns true on the frame it is clicked.
bool ui_button(Rectangle r, const char *label, bool highlighted);

// True while any slider is being dragged (lets other mouse handlers yield).
bool ui_dragging(void);

#endif // UI_H
