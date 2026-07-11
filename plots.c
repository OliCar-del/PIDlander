#include "plots.h"

#include <complex.h>
#include <math.h>

#include "raylib.h"

#include "util.h"

// ---------- polynomial machinery ----------

// a*s^2 + b*s + c = 0
static void solve_quadratic(double a, double b, double c, double re[2], double im[2])
{
    double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
        double r = sqrt(disc);
        re[0] = (-b + r) / (2.0 * a); im[0] = 0.0;
        re[1] = (-b - r) / (2.0 * a); im[1] = 0.0;
    } else {
        double r = sqrt(-disc);
        re[0] = re[1] = -b / (2.0 * a);
        im[0] = r / (2.0 * a);
        im[1] = -im[0];
    }
}

// a*s^3 + b*s^2 + c*s + d = 0 (a != 0), via depressed cubic + Cardano/trig.
static void solve_cubic(double a, double b, double c, double d,
                        double re[3], double im[3])
{
    double p = b / a, q = c / a, r = d / a;
    double P = q - p * p / 3.0;
    double Q = 2.0 * p * p * p / 27.0 - p * q / 3.0 + r;
    double shift = -p / 3.0;
    double D = Q * Q / 4.0 + P * P * P / 27.0;

    if (D > 1e-12) {                 // one real root + complex pair
        double sq = sqrt(D);
        double u = cbrt(-Q / 2.0 + sq), v = cbrt(-Q / 2.0 - sq);
        re[0] = u + v + shift;            im[0] = 0.0;
        re[1] = -(u + v) / 2.0 + shift;   im[1] = (u - v) * sqrt(3.0) / 2.0;
        re[2] = re[1];                    im[2] = -im[1];
    } else if (D < -1e-12) {         // three distinct real roots
        double rho = sqrt(-P * P * P / 27.0);
        double arg = -Q / (2.0 * rho);
        if (arg > 1.0) arg = 1.0;
        if (arg < -1.0) arg = -1.0;
        double theta = acos(arg);
        double mag = 2.0 * sqrt(-P / 3.0);
        for (int k = 0; k < 3; k++) {
            re[k] = mag * cos((theta + 2.0 * PI * (double)k) / 3.0) + shift;
            im[k] = 0.0;
        }
    } else {                         // repeated roots
        double u = cbrt(-Q / 2.0);
        re[0] = 2.0 * u + shift;
        re[1] = re[2] = -u + shift;
        im[0] = im[1] = im[2] = 0.0;
    }
}

// Closed-loop poles for m*s^3 + (c+Kd)*s^2 + Kp*s + Ki = 0 (2nd order if Ki=0).
static int poles(double m, double drag, double kp, double ki, double kd,
                 double re[3], double im[3])
{
    if (ki > 1e-6) {
        solve_cubic(m, drag + kd, kp, ki, re, im);
        return 3;
    }
    solve_quadratic(m, drag + kd, kp, re, im);
    return 2;
}

// Generic Routh-Hurwitz test: coefficients in descending powers, degree deg.
// Returns true iff all roots are strictly in the left half-plane.
static bool routh_stable(const double *coef, int deg)
{
    double a[8] = { 0 }, b[8] = { 0 }, c[8] = { 0 };
    int w = deg / 2 + 1;
    for (int i = 0; 2 * i <= deg; i++)     a[i] = coef[2 * i];
    for (int i = 0; 2 * i + 1 <= deg; i++) b[i] = coef[2 * i + 1];
    for (int i = 0; i <= deg; i++)
        if (coef[i] <= 0.0) return false;  // necessary: all same sign
    for (int row = 0; row < deg - 1; row++) {
        if (b[0] <= 0.0) return false;
        for (int j = 0; j < w - 1; j++)
            c[j] = (b[0] * a[j + 1] - a[0] * b[j + 1]) / b[0];
        c[w - 1] = 0.0;
        for (int j = 0; j < w; j++) { a[j] = b[j]; b[j] = c[j]; }
    }
    return b[0] > 0.0;
}

// Exact closed-loop stability for the linear model INCLUDING the derivative
// filter and (for LAG) the actuator: characteristic polynomial
//   s(1+tf s)(1+ta s)(m s^2 + c s) + Kp s(1+tf s) + Ki(1+tf s) + Kd s^2 = 0
static bool loop_stable(float m, float kp, float ki, float kd, ThrustProf prof)
{
    double tf = D_TAU, ta = (prof == TP_LAG) ? ACT_TAU : 0.0;
    double a[6] = {
        ta * tf * (double)m,
        tf * (double)m + ta * (double)m + ta * tf * (double)DRAG,
        (double)m + tf * (double)DRAG + ta * (double)DRAG,
        (double)DRAG + (double)kd + (double)kp * tf,
        (double)kp + (double)ki * tf,
        (double)ki,
    };
    const double *p = a;
    int deg = 5;
    while (deg > 0 && fabs(p[0]) < 1e-12) { p++; deg--; }   // ta or tf = 0
    if (deg > 0 && fabs(p[deg]) < 1e-9) deg--;  // Ki=0: spurious factor of s
    if (deg <= 0 || fabs(p[deg]) < 1e-9) return false;      // pole at origin
    return routh_stable(p, deg);
}

// ---------- root locus ----------

void locus_draw(int lx, int ly, int lw, int lh,
                float m, float kp, float ki, float kd)
{
    const double RE_MIN = -9.0, RE_MAX = 3.0, IM_MAX = 6.0;

    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);

    // unstable half-plane shading + axes
    int x_zero = lx + (int)((0.0 - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
    int y_zero = ly + lh / 2;
    DrawRectangle(x_zero, ly, lx + lw - x_zero, lh, Fade(RED, 0.10f));
    DrawLine(x_zero, ly, x_zero, ly + lh, Fade(RED, 0.6f));
    DrawLine(lx, y_zero, lx + lw, y_zero, Fade(DARKGRAY, 0.8f));

    // branches: sweep Kp with Ki, Kd held at current values
    double re[3], im[3];
    for (int i = 1; i <= 300; i++) {
        double k = (double)KP_TOP * (double)i / 300.0;
        int n = poles(m, DRAG, k, ki, kd, re, im);
        for (int j = 0; j < n; j++) {
            if (re[j] < RE_MIN || re[j] > RE_MAX || fabs(im[j]) > IM_MAX) continue;
            int px = lx + (int)((re[j] - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
            int py = ly + (int)((IM_MAX - im[j]) / (2.0 * IM_MAX) * (double)lh);
            DrawCircle(px, py, 1.2f, Fade(LIME, 0.45f));
        }
    }

    // current poles as X marks; red once in the right half-plane
    int n = poles(m, DRAG, kp, ki, kd, re, im);
    for (int j = 0; j < n; j++) {
        if (re[j] < RE_MIN || re[j] > RE_MAX || fabs(im[j]) > IM_MAX) continue;
        int px = lx + (int)((re[j] - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
        int py = ly + (int)((IM_MAX - im[j]) / (2.0 * IM_MAX) * (double)lh);
        Color col = (re[j] > 1e-6) ? RED : YELLOW;
        DrawLine(px - 5, py - 5, px + 5, py + 5, col);
        DrawLine(px - 5, py + 5, px + 5, py - 5, col);
    }

    DrawText(TextFormat("ROOT LOCUS: poles vs Kp 0..%.0f (Ki, Kd held)", KP_TOP),
             lx + 8, ly + 6, 13, LIGHTGRAY);
    DrawText("s-plane; filter/actuator/sampling ignored", lx + 8, ly + 22, 12, DARKGRAY);
    DrawText("Re=0", x_zero + 4, ly + lh - 16, 12, Fade(RED, 0.8f));
}

// ---------- Bode plot ----------

// Open-loop L(jw) = C(jw)*A(jw)*G(jw) with the derivative filter and (for
// LAG) the actuator included, so the margins are honest about both. SLEW is
// nonlinear and has no transfer function; it is treated as instant here.
// PM/GM are advisory (this loop can be conditionally stable, where single
// crossings mislead); the STABLE/UNSTABLE verdict is exact via Routh.
void bode_draw(int lx, int ly, int lw, int lh, float m,
               float kp, float ki, float kd, ThrustProf prof)
{
    enum { BN = 160 };
    float mag[BN], ph[BN], lg[BN];
    double prev = 0.0;

    for (int i = 0; i < BN; i++) {
        double lgw = -1.5 + 3.5 * (double)i / (double)(BN - 1);  // 0.03..~100 rad/s
        double w = pow(10.0, lgw);
        double complex jw = I * w;
        double complex G = 1.0 / ((double)m * jw * jw + (double)DRAG * jw);
        double complex C = (double)kp + (double)kd * jw / (1.0 + jw * (double)D_TAU);
        if (ki > 1e-6f) C += (double)ki / jw;
        double complex A = (prof == TP_LAG) ? 1.0 / (1.0 + jw * (double)ACT_TAU) : 1.0;
        double complex L = C * A * G;

        mag[i] = (float)(20.0 * log10(cabs(L) + 1e-12));
        double p = carg(L) * 180.0 / PI;
        if (i > 0) {                                   // unwrap
            while (p - prev > 180.0)  p -= 360.0;
            while (p - prev < -180.0) p += 360.0;
        } else {
            // this plant lags at least -90 deg at low frequency; carg() can
            // report ~+180 for a true -180, which would shift the whole
            // unwrapped curve up a turn — pin the first sample negative
            while (p > 0.0) p -= 360.0;
        }
        prev = p;
        ph[i] = (float)p;
        lg[i] = (float)lgw;
    }

    // margins: PM at the last 0 dB crossing, GM at the worst -180 crossing
    float pm = NAN, w_pm = 0.0f, gm = NAN, w_gm = 0.0f;
    for (int i = 1; i < BN; i++) {
        if (mag[i - 1] >= 0.0f && mag[i] < 0.0f) {
            float t = mag[i - 1] / (mag[i - 1] - mag[i]);
            pm = 180.0f + ph[i - 1] + t * (ph[i] - ph[i - 1]);
            w_pm = powf(10.0f, lg[i - 1] + t * (lg[i] - lg[i - 1]));
        }
        if ((ph[i - 1] > -180.0f) != (ph[i] > -180.0f)) {
            float t = (ph[i - 1] + 180.0f) / (ph[i - 1] - ph[i]);
            float g = -(mag[i - 1] + t * (mag[i] - mag[i - 1]));
            if (isnan(gm) || g < gm) {
                gm = g;
                w_gm = powf(10.0f, lg[i - 1] + t * (lg[i] - lg[i - 1]));
            }
        }
    }

    // frame + two subplots (magnitude over phase)
    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);
    int my0 = ly + 38, mh = (lh - 60) / 2;
    int py0 = my0 + mh + 8, phh = mh;

    bool stable = loop_stable(m, kp, ki, kd, prof);
    DrawText(stable ? "BODE - closed loop STABLE (incl. filter & lag)"
                    : "BODE - closed loop UNSTABLE (incl. filter & lag)",
             lx + 8, ly + 6, 13, stable ? GREEN : RED);
    const char *pm_s = isnan(pm) ? "--" : TextFormat("%.0f deg", pm);
    DrawText(TextFormat("phase margin %s @ %.1f   gain margin %s @ %.1f rad/s",
             pm_s, w_pm, isnan(gm) ? "--" : TextFormat("%.1f dB", gm), w_gm),
             lx + 8, ly + 22, 12, LIGHTGRAY);

    // decade gridlines + reference lines
    for (int d = -1; d <= 2; d++) {
        int px = lx + (int)(((float)d + 1.5f) / 3.5f * (float)lw);
        DrawLine(px, my0, px, my0 + mh, Fade(DARKGRAY, 0.5f));
        DrawLine(px, py0, px, py0 + phh, Fade(DARKGRAY, 0.5f));
        DrawText(TextFormat("%g", pow(10.0, d)), px + 3, py0 + phh - 12, 12, DARKGRAY);
    }
    int zero_db = my0 + (int)(60.0f / 120.0f * (float)mh);
    DrawLine(lx, zero_db, lx + lw, zero_db, Fade(SKYBLUE, 0.4f));
    DrawText("0 dB", lx + 4, zero_db - 12, 12, Fade(SKYBLUE, 0.7f));
    int m180 = py0 + (int)(180.0f / 360.0f * (float)phh);
    DrawLine(lx, m180, lx + lw, m180, Fade(GOLD, 0.4f));
    DrawText("-180", lx + 4, m180 - 12, 12, Fade(GOLD, 0.7f));

    // traces
    for (int i = 1; i < BN; i++) {
        float x0 = (float)lx + (lg[i - 1] + 1.5f) / 3.5f * (float)lw;
        float x1 = (float)lx + (lg[i] + 1.5f) / 3.5f * (float)lw;
        float m0 = clampf(mag[i - 1], -60.0f, 60.0f);
        float m1 = clampf(mag[i],     -60.0f, 60.0f);
        DrawLineV((Vector2){ x0, (float)my0 + (60.0f - m0) / 120.0f * (float)mh },
                  (Vector2){ x1, (float)my0 + (60.0f - m1) / 120.0f * (float)mh }, SKYBLUE);
        float p0 = clampf(ph[i - 1], -360.0f, 0.0f);
        float p1 = clampf(ph[i],     -360.0f, 0.0f);
        DrawLineV((Vector2){ x0, (float)py0 + (0.0f - p0) / 360.0f * (float)phh },
                  (Vector2){ x1, (float)py0 + (0.0f - p1) / 360.0f * (float)phh }, GOLD);
    }
    if (prof == TP_SLEW)
        DrawText("slew is nonlinear: treated as instant", lx + lw - 230, ly + lh - 14, 12, ORANGE);
}

// ---------- phase portrait ----------

void phase_draw(const Chart *c, int lx, int ly, int lw, int lh)
{
    const float E_MAX = 60.0f, V_MAX = 40.0f;

    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);
    int cx = lx + lw / 2, cy = ly + lh / 2;
    DrawLine(lx, cy, lx + lw, cy, Fade(DARKGRAY, 0.6f));
    DrawLine(cx, ly, cx, ly + lh, Fade(DARKGRAY, 0.6f));
    DrawLine(cx - 5, cy, cx + 5, cy, GOLD);            // target equilibrium
    DrawLine(cx, cy - 5, cx, cy + 5, GOLD);

    for (int i = 1; i < c->count; i++) {
        int i0 = (c->head - c->count + i - 1 + 2 * CHART_N) % CHART_N; 
        int i1 = (i0 + 1) % CHART_N;
        float e0 = clampf(c->sp[i0] - c->y[i0], -E_MAX, E_MAX);
        float e1 = clampf(c->sp[i1] - c->y[i1], -E_MAX, E_MAX);
        float v0 = clampf(c->v[i0], -V_MAX, V_MAX);
        float v1 = clampf(c->v[i1], -V_MAX, V_MAX);
        float alpha = 0.10f + 0.85f * (float)i / (float)c->count;
        DrawLineV((Vector2){ (float)lx + (e0 + E_MAX) / (2 * E_MAX) * (float)lw,
                             (float)ly + (V_MAX - v0) / (2 * V_MAX) * (float)lh },
                  (Vector2){ (float)lx + (e1 + E_MAX) / (2 * E_MAX) * (float)lw,
                             (float)ly + (V_MAX - v1) / (2 * V_MAX) * (float)lh },
                  Fade(SKYBLUE, alpha));
    }
    if (c->count > 0) {
        int il = (c->head - 1 + CHART_N) % CHART_N;
        float e = clampf(c->sp[il] - c->y[il], -E_MAX, E_MAX);
        float v = clampf(c->v[il], -V_MAX, V_MAX);
        DrawCircleV((Vector2){ (float)lx + (e + E_MAX) / (2 * E_MAX) * (float)lw,
                               (float)ly + (V_MAX - v) / (2 * V_MAX) * (float)lh },
                    3.5f, RAYWHITE);
    }
    DrawText("PHASE PORTRAIT: error (x) vs velocity (y), last 20 s",
             lx + 8, ly + 6, 13, LIGHTGRAY);
    DrawText("spiral into the crosshair = damped convergence",
             lx + 8, ly + 22, 12, DARKGRAY);
}

// ---------- predicted step response ----------

void step_predict(StepPred *sp, float m, float kp, float ki, float kd,
                  ThrustProf prof)
{
    const float dt = 1.0f / 120.0f;
    // Round, don't truncate: STEP_T/dt in float is 1199.9995..., and the
    // truncated 1199 made sub=4, so curve[i/sub] ran 60 slots past the end
    // of the array — corrupting whatever the stack placed after it (it was
    // the history chart's altitude ring).
    const int n = (int)(STEP_T / dt + 0.5f);
    const int sub = n / STEP_N;
    float y = 0.0f, v = 0.0f, integ = 0.0f, dfilt = 0.0f, prevy = 0.0f, uact = 0.0f;

    sp->ov_pct = 0.0f;
    sp->settle = -1.0f;
    sp->diverged = false;
    for (int i = 0; i < n; i++) {
        float e = 1.0f - y;
        integ += ki * e * dt;
        float draw = -(y - prevy) / dt;
        prevy = y;
        dfilt += (draw - dfilt) * dt / (D_TAU + dt);
        float ucmd = kp * e + integ + kd * dfilt;
        uact = actuate(uact, ucmd, (prof == TP_LAG) ? TP_LAG : TP_INSTANT, dt);
        v += (uact - DRAG * v) / m * dt;
        y += v * dt;

        if (i % sub == 0 && i / sub < STEP_N) sp->curve[i / sub] = y;
        if (y - 1.0f > sp->ov_pct) sp->ov_pct = y - 1.0f;
        if (fabsf(1.0f - y) > 0.02f) sp->settle = -1.0f;
        else if (sp->settle < 0.0f) sp->settle = (float)i * dt;
        if (fabsf(y) > 100.0f) {
            sp->diverged = true;
            for (int k = (i / sub) + 1; k < STEP_N; k++) sp->curve[k] = y;
            break;
        }
    }
    sp->ov_pct *= 100.0f;
}

void step_draw(const StepPred *sp, int lx, int ly, int lw, int lh)
{
    const float Y_TOP = 1.8f;                      // plotted range 0..1.8

    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);
    DrawText("PREDICTED STEP RESPONSE (linear, no saturation)",
             lx + 8, ly + 6, 13, LIGHTGRAY);
    if (sp->diverged)
        DrawText("UNSTABLE: response diverges", lx + 8, ly + 22, 12, RED);
    else
        DrawText(TextFormat("overshoot %.1f%%   2%%-settle %s", sp->ov_pct,
                 sp->settle < 0.0f ? "> 10 s" : TextFormat("%.1f s", sp->settle)),
                 lx + 8, ly + 22, 12, SKYBLUE);

    int gy = ly + 40, gh = lh - 58;
    int target = gy + (int)((Y_TOP - 1.0f) / Y_TOP * (float)gh);
    DrawLine(lx, target, lx + lw, target, Fade(GOLD, 0.6f));
    DrawText("target", lx + 4, target - 14, 12, Fade(GOLD, 0.8f));
    int band = (int)(0.02f / Y_TOP * (float)gh);
    DrawLine(lx, target - band, lx + lw, target - band, Fade(GOLD, 0.2f));
    DrawLine(lx, target + band, lx + lw, target + band, Fade(GOLD, 0.2f));
    for (int s = 0; s <= (int)STEP_T; s += 2) {
        int px = lx + (int)((float)s / STEP_T * (float)lw);
        DrawLine(px, gy, px, gy + gh, Fade(DARKGRAY, 0.4f));
        DrawText(TextFormat("%ds", s), px + 3, gy + gh + 2, 12, DARKGRAY);
    }

    for (int i = 1; i < STEP_N; i++) {
        float y0 = clampf(sp->curve[i - 1], -0.05f, Y_TOP);
        float y1 = clampf(sp->curve[i],     -0.05f, Y_TOP);
        DrawLineV((Vector2){ (float)lx + (float)(i - 1) / (STEP_N - 1) * (float)lw,
                             (float)gy + (Y_TOP - y0) / Y_TOP * (float)gh },
                  (Vector2){ (float)lx + (float)i / (STEP_N - 1) * (float)lw,
                             (float)gy + (Y_TOP - y1) / Y_TOP * (float)gh },
                  SKYBLUE);
    }
}
