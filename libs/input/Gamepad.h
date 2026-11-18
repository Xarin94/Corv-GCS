// Gamepad.h - game controllers and joysticks through SDL3 (the Gamepad API of
// Chromium in the Electron version).
//
// A device SDL recognises as a gamepad is read in the W3C "standard" layout
// Chromium gave it: axes 0-3 left X, left Y, right X, right Y (up and left
// negative), buttons 0-16 south, east, west, north, left and right shoulder,
// left and right trigger (analog 0..1), back, start, left and right stick,
// d-pad up, down, left, right, guide. A configuration saved by Electron keeps
// its meaning. Any other joystick (a radio transmitter on USB, a flight stick)
// gives its own axes and buttons in device order, as Chromium did.
//
// Read from the GUI thread with SDL's state updated on each read, no event
// loop; device events are also delivered while the window is in the
// background. Without SDL3 in the build there are no devices.
#pragma once

#include <QString>
#include <vector>

namespace corv {

class Gamepad
{
public:
    struct Device {
        unsigned id = 0;         // SDL instance id, stable while plugged
        QString name;
        int axes = 0, buttons = 0;
        bool standard = false;   // read in the standard layout
    };
    struct State {
        std::vector<double> axes;      // -1..1
        std::vector<double> buttons;   // 0..1
    };

    static bool available();

    Gamepad();
    ~Gamepad();
    Gamepad(const Gamepad &) = delete;
    Gamepad &operator=(const Gamepad &) = delete;

    /// The devices plugged now.
    std::vector<Device> devices();
    bool open(unsigned id);
    void close();
    bool isOpen() const { return m_open; }
    const Device &device() const { return m_device; }
    /// The state now; false once the device is gone (it is closed then).
    bool read(State &out);

private:
    bool init();
    bool m_inited = false, m_open = false;
    void *m_gamepad = nullptr;    // SDL_Gamepad *
    void *m_joystick = nullptr;   // SDL_Joystick * (non-standard devices)
    Device m_device;
};

} // namespace corv
