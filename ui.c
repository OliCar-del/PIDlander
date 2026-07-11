// ui.c — implementation of the immediate-mode slider and button.

#include "ui.h"

#include "config.h"

// ---- slider geometry, relative to the (x, y) passed by the caller ----
#define SL_LABEL_FS   FS_BODY        // label / value font size
#define SL_TRACK_DY   24             // track centerline offset below y [px]
#define SL_TRACK_H    4              // track thickness [px]
#define SL_KNOB_R     7.0f           // knob radius [px]
#define SL_HIT_DY     16             // clickable band: from y+16 ...
#define SL_HIT_H      18             // ... spanning 18 px of height

// The one piece of retained state: which slider id currently owns the mouse
// drag, or -1 when none does. Buttons need no retained state because a click
// is detected on a single frame.
static int active_id = -1;

float ui_slider(int id, int x, int y, int w, const char *label,
                float value, float lo, float hi, const char *fmt)
{
    Vector2 mouse = GetMousePosition();
    Rectangle hit = { (float)x, (float)(y + SL_HIT_DY), (float)w, SL_HIT_H };

    // grab: a press inside the hit band captures the drag for this id
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, hit))
        active_id = id;
    // release: letting go of the button releases whichever slider held it
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) && active_id == id)
        active_id = -1;
    // drag: map pointer x onto [lo, hi], clamped to the track ends
    if (active_id == id) {
        float t = (mouse.x - (float)x) / (float)w;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        value = lo + t * (hi - lo);
    }

    // label on the left, formatted value right-aligned on the same line
    DrawText(label, x, y, SL_LABEL_FS, LIGHTGRAY);
    const char *value_text = TextFormat(fmt, value);
    DrawText(value_text, x + w - MeasureText(value_text, SL_LABEL_FS), y,
             SL_LABEL_FS, RAYWHITE);

    // track: dark full-length bar with the filled portion up to the knob
    int track_y = y + SL_TRACK_DY;
    float t = (value - lo) / (hi - lo);
    int knob_x = x + (int)(t * (float)w);
    DrawRectangle(x, track_y - SL_TRACK_H / 2, w, SL_TRACK_H, DARKGRAY);
    DrawRectangle(x, track_y - SL_TRACK_H / 2, knob_x - x, SL_TRACK_H, SKYBLUE);
    DrawCircle(knob_x, track_y, SL_KNOB_R,
               (active_id == id) ? RAYWHITE : SKYBLUE);
    return value;
}

bool ui_button(Rectangle r, const char *label, bool highlighted)
{
    Vector2 mouse = GetMousePosition();
    bool hover = CheckCollisionPointRec(mouse, r);
    bool clicked = hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    // fill brightness communicates state: highlighted > hovered > idle
    Color fill = highlighted ? Fade(SKYBLUE, 0.35f)
               : hover       ? Fade(GRAY, 0.35f)
                             : Fade(GRAY, 0.15f);
    DrawRectangleRec(r, fill);
    DrawRectangleLinesEx(r, 1.0f, highlighted ? SKYBLUE : GRAY);

    // center the label inside the button rectangle
    int text_w = MeasureText(label, FS_BODY);
    DrawText(label,
             (int)(r.x + (r.width - (float)text_w) / 2.0f),
             (int)(r.y + (r.height - (float)FS_BODY) / 2.0f),
             FS_BODY, highlighted ? RAYWHITE : LIGHTGRAY);
    return clicked;
}

bool ui_dragging(void)
{
    return active_id != -1;
}
