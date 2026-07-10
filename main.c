// main.c — Phase 0 setup check for the 1D PID lander project.
// If this compiles and opens a window at 60 FPS, your toolchain is ready.
//
// Linux/macOS:  make        (see Makefile)
// Manual:       gcc main.c -o lander -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

#include "raylib.h"

int main(void)
{
    const int W = 800, H = 600;
    InitWindow(W, H, "PID Lander — setup check");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground((Color){ 10, 14, 26, 255 });

        DrawRectangle(W/2 - 15, H/2 - 20, 30, 40, RAYWHITE);   // the craft (static for now)
        DrawLine(0, H - 60, W, H - 60, GRAY);                  // the ground
        DrawText("raylib is working — ready for Phase 1", 20, 20, 20, GREEN);
        DrawFPS(W - 100, 20);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
