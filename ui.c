#include "ui.h"

static int active_id = -1;   // slider currently captured by the mouse

float ui_slider(int id, int x, int y, int w, const char *label,
                float value, float lo, float hi, const char *fmt)
{
    Vector2 mp = GetMousePosition();
    Rectangle hit = { (float)x, (float)y + 16, (float)w, 18 };

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mp, hit))
        active_id = id;
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) && active_id == id)
        active_id = -1;
    if (active_id == id) {
        float t = (mp.x - (float)x) / (float)w;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        value = lo + t * (hi - lo);
    }

    DrawText(label, x, y, 16, LIGHTGRAY);
    const char *vs = TextFormat(fmt, value);
    DrawText(vs, x + w - MeasureText(vs, 16), y, 16, RAYWHITE);

    int ty = y + 24;                         // track centerline
    float t = (value - lo) / (hi - lo);
    int kx = x + (int)(t * (float)w);
    DrawRectangle(x, ty - 2, w, 4, DARKGRAY);
    DrawRectangle(x, ty - 2, kx - x, 4, SKYBLUE);
    DrawCircle(kx, ty, 7.0f, (active_id == id) ? RAYWHITE : SKYBLUE);
    return value;
}

bool ui_button(Rectangle r, const char *label, bool highlighted)
{
    Vector2 mp = GetMousePosition();
    bool hover = CheckCollisionPointRec(mp, r);
    bool clicked = hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    Color fill = highlighted ? Fade(SKYBLUE, 0.35f)
               : hover       ? Fade(GRAY, 0.35f)
                             : Fade(GRAY, 0.15f);
    DrawRectangleRec(r, fill);
    DrawRectangleLinesEx(r, 1.0f, highlighted ? SKYBLUE : GRAY);
    int tw = MeasureText(label, 16);
    DrawText(label, (int)(r.x + (r.width - (float)tw) / 2.0f),
             (int)(r.y + (r.height - 16.0f) / 2.0f), 16,
             highlighted ? RAYWHITE : LIGHTGRAY);
    return clicked;
}

bool ui_dragging(void)
{
    return active_id != -1;
}
