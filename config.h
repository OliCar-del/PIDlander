// config.h — single source of truth for every tunable constant.
//
// Sections:
//   1. physics       - plant parameters and simulation timing
//   2. actuator      - thrust response models
//   3. controller    - PID defaults, slider ceilings, filter constant
//   4. wind          - gust generator timing and amplitude
//   5. window layout - pixel geometry of every panel
//   6. instruments   - history chart, analysis plot, sequence editor
//   7. style         - shared colors and font sizes
//
// Conventions used throughout the project:
//   - altitude y is in meters, up positive, 0 = ground
//   - velocity v in m/s, thrust u in newtons, time in seconds
//   - screen coordinates are pixels, origin top-left, y down

#ifndef CONFIG_H
#define CONFIG_H

// raylib defines PI when it is included; provide it for headless builds too.
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ---------- 1. physics ----------
#define SIM_DT      (1.0f / 120.0f)  // fixed physics timestep [s]
#define PHYS_HZ     120              // physics steps per second (1/SIM_DT)
#define MASS_DEF    1.0f             // default craft mass [kg]
#define G_DEF       9.81f            // default gravity [m/s^2]
#define DRAG        0.15f            // linear drag coefficient [N*s/m]
#define UMAX_DEF    50.0f            // default max thrust [N]
#define UMAX_TOP    100.0f           // max-thrust slider ceiling [N]
#define WORLD_H     100.0f           // visible altitude range [m]
#define START_Y     50.0f            // spawn altitude [m]
#define VCRASH_DEF  5.0f             // default crash touchdown speed [m/s]
#define FRAME_CLAMP 0.25f            // longest frame the accumulator accepts [s]

// ---------- 2. actuator ----------
#define ACT_TAU     0.2f             // LAG: first-order time constant [s]
#define ACT_SLEW    60.0f            // SLEW: max thrust rate of change [N/s]

// ---------- 3. controller ----------
#define KP_DEF      4.0f             // default proportional gain [N/m]
#define KI_DEF      0.5f             // default integral gain [N/(m*s)]
#define KD_DEF      4.0f             // default derivative gain [N*s/m]
#define KP_TOP      50.0f            // slider ceilings for each gain
#define KI_TOP      20.0f
#define KD_TOP      30.0f
#define D_TAU       0.05f            // derivative low-pass time constant [s]
#define SP_START    50.0f            // initial setpoint [m]
#define SP_RATE     15.0f            // setpoint slew from arrow keys [m/s]
#define NOISE_TOP   2.0f             // sensor noise sigma slider ceiling [m]

// ---------- 4. wind gusts ----------
#define GUST_GAP_MIN  2.0f           // quiet time between gusts [s]
#define GUST_GAP_MAX  6.0f
#define GUST_DUR_MIN  1.0f           // duration of one gust pulse [s]
#define GUST_DUR_MAX  2.0f
#define WIND_AMP_MAX  50.0f          // slider ceiling for peak gust force [N]

// ---------- 5. window layout ----------
// Overall: title bar across the top; below it three columns (telemetry
// panel, flight view, controller panel); along the bottom the history
// chart, analysis plot, and target sequence editor.
#define SCREEN_W    1600
#define SCREEN_H    1000
#define TITLE_H     48               // title bar height

#define LPAN_X      8                // telemetry panel (left column)
#define LPAN_Y      56
#define LPAN_W      356
#define LPAN_H      640

#define VIEW_X0     372              // flight view (center column)
#define VIEW_X1     1128
#define VIEW_Y0     56
#define VIEW_Y1     696
#define MARGIN_PX   40               // view padding above 100 m and below 0 m
#define CRAFT_W_PX  30               // craft body size on screen
#define CRAFT_H_PX  40

#define RPAN_X      1136             // controller panel (right column)
#define RPAN_Y      56
#define RPAN_W      456
#define RPAN_H      640

// ---------- 6. instruments ----------
// History chart (bottom left): ring buffer of CHART_N samples at CHART_HZ,
// so the window shows the last CHART_N / CHART_HZ = 20 seconds.
#define CHART_N     600              // samples held in the ring buffer
#define CHART_HZ    30               // sample rate [Hz]
#define CHART_X     8
#define CHART_Y     704
#define CHART_W     852
#define CHART_HGT   240

// Analysis plot slot (bottom middle): locus / Bode / phase / step share it.
#define PLOT_X      868
#define PLOT_Y      704
#define PLOT_W      356
#define PLOT_HGT    240

// Target sequence editor (bottom right): panel frame and inner graph area.
#define SEQ_MAX     10               // maximum number of profile points
#define SEQ_PX      1232
#define SEQ_PY      704
#define SEQ_PW      360
#define SEQ_PH      240
#define SEQ_GX      1242             // inner graph (time vs height)
#define SEQ_GY      770
#define SEQ_GW      340
#define SEQ_GH      164

#define SCORE_KEEP  4                // playback score rows kept and shown
#define WPART       28               // wind streak particles in the view
#define STEP_N      240              // samples in the predicted step curve
#define STEP_T      10.0f            // predicted step horizon [s]

// ---------- 7. style ----------
// Colors are macro initializers, usable anywhere raylib.h is included.
#define COL_WINDOW_BG  (Color){ 10, 14, 26, 255 }   // main window background
#define COL_TITLE_BG   (Color){ 15, 21, 40, 255 }   // title bar background
#define COL_PLOT_BG    (Color){ 16, 21, 36, 255 }   // instrument backgrounds
#define COL_WRECK      (Color){ 90, 90, 95, 255 }   // crashed hull
#define COL_WRECK_DARK (Color){ 70, 70, 75, 255 }   // scattered wreck pieces

// Font sizes [px], smallest to largest.
#define FS_TINY     12               // axis ticks, point numbers
#define FS_SMALL    14               // captions, legends, hints
#define FS_BODY     16               // panel body text, buttons
#define FS_MED      18               // emphasized values
#define FS_LARGE    22               // telemetry readouts
#define FS_TITLE    30               // game title
#define FS_HUGE     32               // status word, pause banner

#endif // CONFIG_H
