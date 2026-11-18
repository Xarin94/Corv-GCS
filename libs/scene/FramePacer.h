// FramePacer.h - the 3D view's frame cap (SYS CONFIG -> RENDER FPS CAP,
// renderFpsCap; the render-loop cap of js/main.js).
//
// Every frame of the 3D view costs the GPU the terrain, the satellite imagery
// and every overlay, and the window a recomposite. The Electron version drew
// 3D + HUD at most 60 times a second (30 in eco mode); uncapped, the view
// follows the display - 120 or 144 times a second on a laptop panel, twice the
// work and the battery for no visible gain in a flight view.
//
// The cap is a whole number of vsync periods per frame - 60 fps is every
// second vblank at 120 Hz and every one at 60 Hz, 30 fps every second one at
// 60 Hz - so frames stay evenly paced; a cap that does not divide the refresh
// rate rounds to the nearest divisor (72 fps at 144 Hz, as in the Electron
// version). After each frame the next one is due one interval after this one
// was due (a phase-locked grid: the rate does not creep) and is requested half
// a vsync early: the render thread is idle by then, draws at once and presents
// at the vblank the frame was meant for. When the vblank times are known (DWM
// on Windows) the grid is snapped to them, so a timer firing a millisecond
// early or late never moves a frame to the neighbouring vblank.
//
// Pure arithmetic on times in ms (monoMs()); the 3D view owns the timer.
#pragma once

namespace corv {

class FramePacer
{
public:
    /// Vblank timing of the display, on the clock of the times passed in.
    struct Vsync {
        double periodMs = 0;   // 0: unknown
        double vblankMs = 0;   // the time of one vblank
    };

    /// At most this many frames a second; 0: every vblank.
    void setCap(int fps) { m_cap = fps; }
    int cap() const { return m_cap; }
    /// Forget the schedule; the next frame starts a new grid.
    void reset() { m_dueMs = -1; }

    /// A frame was just drawn: when to ask for the next one (>= nowMs). refreshHz
    /// is the display's rate, used when vs has no period.
    double next(double nowMs, double refreshHz, const Vsync &vs);
    double next(double nowMs, double refreshHz);
    /// The vblank the frame asked for last is meant for (-1: none yet).
    double dueMs() const { return m_dueMs; }

    /// The display's vblank timing from the OS on monoMs()'s clock (DWM, the
    /// primary display); empty elsewhere.
    static Vsync systemVsync();

private:
    int m_cap = 60;
    double m_dueMs = -1;
};

// Out of the class: GCC cannot take a default `Vsync{}` argument while
// FramePacer, and so Vsync's member initialisers, are still incomplete
inline double FramePacer::next(double nowMs, double refreshHz) { return next(nowMs, refreshHz, Vsync{}); }

} // namespace corv
