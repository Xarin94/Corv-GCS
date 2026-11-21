#include "input/Gamepad.h"

#include "core/DebugLog.h"

#include <algorithm>

#ifdef CORV_HAVE_SDL3
#include <SDL3/SDL.h>
#endif

using namespace Qt::StringLiterals;

namespace corv {

#ifdef CORV_HAVE_SDL3

namespace {
constexpr int kStandardAxes = 4, kStandardButtons = 17;

double axisValue(Sint16 v) { return std::clamp(double(v) / 32767.0, -1.0, 1.0); }

/// The W3C standard layout, button by button
const SDL_GamepadButton kButtons[kStandardButtons] = {
    SDL_GAMEPAD_BUTTON_SOUTH,       SDL_GAMEPAD_BUTTON_EAST,          SDL_GAMEPAD_BUTTON_WEST,       SDL_GAMEPAD_BUTTON_NORTH,
    SDL_GAMEPAD_BUTTON_LEFT_SHOULDER, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, SDL_GAMEPAD_BUTTON_INVALID, SDL_GAMEPAD_BUTTON_INVALID,
    SDL_GAMEPAD_BUTTON_BACK,        SDL_GAMEPAD_BUTTON_START,         SDL_GAMEPAD_BUTTON_LEFT_STICK, SDL_GAMEPAD_BUTTON_RIGHT_STICK,
    SDL_GAMEPAD_BUTTON_DPAD_UP,     SDL_GAMEPAD_BUTTON_DPAD_DOWN,     SDL_GAMEPAD_BUTTON_DPAD_LEFT,  SDL_GAMEPAD_BUTTON_DPAD_RIGHT,
    SDL_GAMEPAD_BUTTON_GUIDE};
} // namespace

bool Gamepad::available() { return true; }

bool Gamepad::init()
{
    if (m_inited) return true;
    // Keep reading while another window has the focus: the override must not drop
    // because the operator clicked elsewhere (Chromium hid the pads then)
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    if (!SDL_InitSubSystem(SDL_INIT_GAMEPAD)) {
        LOG_W("joystick") << "SDL gamepad subsystem:" << SDL_GetError();
        return false;
    }
    SDL_SetJoystickEventsEnabled(false);   // state read on demand, no event queue
    m_inited = true;
    LOG_I("joystick") << "SDL" << SDL_MAJOR_VERSION << "." << SDL_MINOR_VERSION << "." << SDL_MICRO_VERSION << "gamepad input";
    return true;
}

Gamepad::Gamepad() = default;

Gamepad::~Gamepad()
{
    close();
    if (m_inited) SDL_QuitSubSystem(SDL_INIT_GAMEPAD);
}

std::vector<Gamepad::Device> Gamepad::devices()
{
    std::vector<Device> out;
    if (!init()) return out;
    SDL_UpdateJoysticks();
    int n = 0;
    SDL_JoystickID *ids = SDL_GetJoysticks(&n);
    for (int i = 0; ids && i < n; ++i) {
        Device d;
        d.id = ids[i];
        d.standard = SDL_IsGamepad(ids[i]);
        if (d.standard) {
            d.name = QString::fromUtf8(SDL_GetGamepadNameForID(ids[i]));
            d.axes = kStandardAxes;
            d.buttons = kStandardButtons;
        } else {
            d.name = QString::fromUtf8(SDL_GetJoystickNameForID(ids[i]));
            if (m_open && m_device.id == d.id) {
                d.axes = m_device.axes;
                d.buttons = m_device.buttons;
            } else if (SDL_Joystick *j = SDL_OpenJoystick(ids[i])) {
                d.axes = SDL_GetNumJoystickAxes(j);
                d.buttons = SDL_GetNumJoystickButtons(j);
                SDL_CloseJoystick(j);
            }
        }
        if (d.name.isEmpty()) d.name = u"Joystick %1"_s.arg(d.id);
        out.push_back(d);
    }
    SDL_free(ids);
    return out;
}

bool Gamepad::open(unsigned id)
{
    if (m_open && m_device.id == id) return true;
    close();
    for (const Device &d : devices()) {
        if (d.id != id) continue;
        if (d.standard) m_gamepad = SDL_OpenGamepad(SDL_JoystickID(id));
        else m_joystick = SDL_OpenJoystick(SDL_JoystickID(id));
        if (!m_gamepad && !m_joystick) {
            LOG_W("joystick") << "cannot open" << d.name << ":" << SDL_GetError();
            return false;
        }
        m_device = d;
        m_open = true;
        LOG_I("joystick") << "opened" << d.name << (d.standard ? "(standard layout)" : "") << d.axes << "axes," << d.buttons << "buttons";
        return true;
    }
    return false;
}

void Gamepad::close()
{
    if (m_gamepad) SDL_CloseGamepad(static_cast<SDL_Gamepad *>(m_gamepad));
    if (m_joystick) SDL_CloseJoystick(static_cast<SDL_Joystick *>(m_joystick));
    m_gamepad = m_joystick = nullptr;
    m_open = false;
}

bool Gamepad::read(State &out)
{
    if (!m_open) return false;
    SDL_UpdateJoysticks();
    if (m_gamepad) {
        auto *g = static_cast<SDL_Gamepad *>(m_gamepad);
        if (!SDL_GamepadConnected(g)) {
            close();
            return false;
        }
        out.axes = {axisValue(SDL_GetGamepadAxis(g, SDL_GAMEPAD_AXIS_LEFTX)), axisValue(SDL_GetGamepadAxis(g, SDL_GAMEPAD_AXIS_LEFTY)),
                    axisValue(SDL_GetGamepadAxis(g, SDL_GAMEPAD_AXIS_RIGHTX)), axisValue(SDL_GetGamepadAxis(g, SDL_GAMEPAD_AXIS_RIGHTY))};
        out.buttons.assign(kStandardButtons, 0.0);
        for (int b = 0; b < kStandardButtons; ++b) {
            if (b == 6 || b == 7) {
                // Triggers: analog buttons 0..1
                const Sint16 t = SDL_GetGamepadAxis(g, b == 6 ? SDL_GAMEPAD_AXIS_LEFT_TRIGGER : SDL_GAMEPAD_AXIS_RIGHT_TRIGGER);
                out.buttons[std::size_t(b)] = std::clamp(double(t) / 32767.0, 0.0, 1.0);
            } else {
                out.buttons[std::size_t(b)] = SDL_GetGamepadButton(g, kButtons[b]) ? 1.0 : 0.0;
            }
        }
        return true;
    }
    auto *j = static_cast<SDL_Joystick *>(m_joystick);
    if (!SDL_JoystickConnected(j)) {
        close();
        return false;
    }
    out.axes.resize(std::size_t(std::max(0, m_device.axes)));
    for (int a = 0; a < m_device.axes; ++a) out.axes[std::size_t(a)] = axisValue(SDL_GetJoystickAxis(j, a));
    out.buttons.resize(std::size_t(std::max(0, m_device.buttons)));
    for (int b = 0; b < m_device.buttons; ++b) out.buttons[std::size_t(b)] = SDL_GetJoystickButton(j, b) ? 1.0 : 0.0;
    return true;
}

#else

bool Gamepad::available() { return false; }
bool Gamepad::init() { return false; }
Gamepad::Gamepad() = default;
Gamepad::~Gamepad() = default;
std::vector<Gamepad::Device> Gamepad::devices() { return {}; }
bool Gamepad::open(unsigned) { return false; }
void Gamepad::close() {}
bool Gamepad::read(State &) { return false; }

#endif

} // namespace corv
