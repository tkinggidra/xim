#include <Arduino.h>

// XIM firmware: mouse + keyboard in on the USB-A port (via a hub),
// gamepad out on USB-C.
// Core 0 pretends to be a gamepad. Core 1 reads the mouse/keyboard.
//
// The mouse's SPEED (smoothed, curved) controls the right stick.
//
// Golden rule: the gamepad loop on core 0 must never wait on anything.

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include <pico/mutex.h>
#include <math.h>

// ---------- aim tuning ----------
const float SENSITIVITY = 10.0f;  // how quickly mouse speed fills the stick
const float SMOOTH_MS   = 10.0f;  // keep >= your mouse's report gap (10 ms at 100 Hz)
const float STOP_SPEED  = 0.03f;  // slower than this (counts/ms) = stick centred
const int   DEADZONE    = 40;     // the game's deadzone (measure with the P test)
const int   MAX_STICK   = 118;    // stay under full push, where acceleration kicks in
const float CURVE       = 1.6f;   // 1.0 = straight line; higher = more stick for small moves

// ---------- general ----------
const int   FRAME_MS    = 4;      // send a gamepad report every 4 ms
const bool  DEBUG       = true;   // print a status line every second
const uint32_t CORE1_TIMEOUT_MS = 300;  // reboot if core 1 is silent this long

// ================= BUTTON MAPPING =================
// Edit these tables to change the layout.

// Normal keys -> controller buttons
struct KeyBind { uint8_t key; uint32_t button; };
const KeyBind KEY_BINDS[] = {
  { HID_KEY_SPACE,  GAMEPAD_BUTTON_A      },  // jump / confirm
  { HID_KEY_C,      GAMEPAD_BUTTON_B      },  // crouch / back
  { HID_KEY_R,      GAMEPAD_BUTTON_X      },  // reload
  { HID_KEY_F,      GAMEPAD_BUTTON_Y      },  // interact / swap
  { HID_KEY_Q,      GAMEPAD_BUTTON_TL     },  // LB
  { HID_KEY_E,      GAMEPAD_BUTTON_TR     },  // RB
  { HID_KEY_V,      GAMEPAD_BUTTON_THUMBR },  // right stick click (melee)
  { HID_KEY_TAB,    GAMEPAD_BUTTON_SELECT },  // Back / View
  { HID_KEY_ESCAPE, GAMEPAD_BUTTON_START  },  // Start / Menu
};

// Modifier keys (Shift/Ctrl/Alt) -> controller buttons
struct ModBind { uint8_t mod_bit; uint32_t button; };
const ModBind MOD_BINDS[] = {
  { KEYBOARD_MODIFIER_LEFTSHIFT, GAMEPAD_BUTTON_THUMBL },  // sprint
  { KEYBOARD_MODIFIER_LEFTCTRL,  GAMEPAD_BUTTON_B      },  // crouch
};

// Mouse buttons -> controller buttons
// (bit 0x01 left, 0x02 right, 0x04 middle, 0x08 back side, 0x10 front side)
struct MouseBind { uint8_t mouse_bit; uint32_t button; };
const MouseBind MOUSE_BINDS[] = {
  { 0x01, GAMEPAD_BUTTON_TR2 },  // left click  = RT (fire)
  { 0x02, GAMEPAD_BUTTON_TL2 },  // right click = LT (aim)
  { 0x04, GAMEPAD_BUTTON_Y   },  // middle      = Y
  { 0x08, GAMEPAD_BUTTON_TL  },  // mouse 4     = LB
  { 0x10, GAMEPAD_BUTTON_TR  },  // mouse 5     = RB
};

// D-pad keys (each list: any of these keys = that direction)
const uint8_t DPAD_UP[]    = { HID_KEY_ARROW_UP,    HID_KEY_1 };
const uint8_t DPAD_RIGHT[] = { HID_KEY_ARROW_RIGHT, HID_KEY_2 };
const uint8_t DPAD_DOWN[]  = { HID_KEY_ARROW_DOWN,  HID_KEY_3 };
const uint8_t DPAD_LEFT[]  = { HID_KEY_ARROW_LEFT,  HID_KEY_4 };

// Stick keys: full push while held
const uint8_t LSTICK_UP = HID_KEY_W, LSTICK_LEFT = HID_KEY_A,
              LSTICK_DOWN = HID_KEY_S, LSTICK_RIGHT = HID_KEY_D;
const uint8_t RSTICK_UP = HID_KEY_I, RSTICK_LEFT = HID_KEY_J,
              RSTICK_DOWN = HID_KEY_K, RSTICK_RIGHT = HID_KEY_L;

// Hold this key to slowly sweep the right stick (deadzone finder)
const uint8_t DEADZONE_FINDER_KEY = HID_KEY_P;

// ---------- device side: what the PC sees ----------
uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_GAMEPAD() };
Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

// ---------- host side: mouse + keyboard ----------
Adafruit_USBH_Host USBHost;

// Shared between the cores, so only touched while holding the lock.
auto_init_mutex(shared_lock);
int32_t  mouse_dx = 0, mouse_dy = 0;  // movement since last frame
uint8_t  mouse_buttons = 0;
uint8_t  keys[6] = {0};               // keyboard keys currently held
uint8_t  key_mods = 0;                // Shift/Ctrl/Alt bits
uint32_t reports_received = 0;        // debug counters
uint32_t request_failures = 0;
uint32_t mount_count = 0;             // every HID interface seen
uint32_t listen_count = 0;            // the ones we actually read
volatile uint32_t core1_heartbeat = 0;  // core 1 adds 1 every loop
volatile uint8_t  core1_stage = 0;      // where core 1 is (for freeze reports)

// The mouse/keyboard interfaces we're listening to.
// (Only touched on core 1, so no lock needed.)
struct Listener { uint8_t dev, inst; bool active, needs_retry; };
Listener listeners[16];

int16_t read_i16(uint8_t const* p) {
  return (int16_t)(p[0] | (p[1] << 8));
}

// ================= CORE 0: gamepad =================

void setup() {
  Serial.begin(115200);
  usb_hid.setPollInterval(1);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

bool keyHeld(uint8_t code, const uint8_t k[6]) {
  for (int i = 0; i < 6; i++) if (k[i] == code) return true;
  return false;
}

// True if any key in the list is held.
template <size_t N>
bool anyHeld(const uint8_t (&list)[N], const uint8_t k[6]) {
  for (size_t i = 0; i < N; i++) if (keyHeld(list[i], k)) return true;
  return false;
}

// Four D-pad directions -> one hat value.
uint8_t dpadToHat(bool up, bool right, bool down, bool left) {
  if (up && down)    up = down = false;      // opposites cancel
  if (left && right) left = right = false;
  if (up && right)   return GAMEPAD_HAT_UP_RIGHT;
  if (down && right) return GAMEPAD_HAT_DOWN_RIGHT;
  if (down && left)  return GAMEPAD_HAT_DOWN_LEFT;
  if (up && left)    return GAMEPAD_HAT_UP_LEFT;
  if (up)            return GAMEPAD_HAT_UP;
  if (right)         return GAMEPAD_HAT_RIGHT;
  if (down)          return GAMEPAD_HAT_DOWN;
  if (left)          return GAMEPAD_HAT_LEFT;
  return GAMEPAD_HAT_CENTERED;
}

// Smoothed mouse speed (counts per ms) -> stick position.
//  1. speed x SENSITIVITY, as a fraction (0..1) of the usable stick range
//  2. bend it by CURVE, to cancel out the game's own response curve
//  3. place it between the game's deadzone and our max
int8_t speedToStick(float v) {
  if (fabsf(v) < STOP_SPEED) return 0;           // basically still: centre
  float range = (float)(MAX_STICK - DEADZONE);
  float n = fabsf(v) * SENSITIVITY / range;      // 0 = barely moving, 1 = max
  if (n > 1.0f) n = 1.0f;
  n = powf(n, 1.0f / CURVE);                     // small moves get a boost
  float s = DEADZONE + n * range;
  return (int8_t)(v > 0 ? s : -s);
}

// Safety net: if core 1 stops looping, reboot the whole board.
void checkCore1() {
  static uint32_t last_beat = 0, last_change = 0;
  uint32_t beat = core1_heartbeat, now = millis();
  if (beat != last_beat) {
    last_beat = beat;
    last_change = now;
  } else if (last_change && now - last_change > CORE1_TIMEOUT_MS) {
    if (Serial && Serial.availableForWrite() > 64) {
      Serial.printf("core 1 frozen at stage %u - rebooting\n", core1_stage);
      Serial.flush();
    }
    rp2040.reboot();
  }
}

void printStatus() {
  static uint32_t last_print = 0, last_reports = 0, frames_sent = 0;
  static uint32_t last_frames = 0, last_beat = 0;
  frames_sent++;
  if (!DEBUG || millis() - last_print < 1000) return;
  if (!Serial || Serial.availableForWrite() < 220) return;  // never block
  last_print = millis();

  mutex_enter_blocking(&shared_lock);
  uint32_t r = reports_received, f = request_failures;
  uint32_t m = mount_count, l = listen_count;
  mutex_exit(&shared_lock);
  uint32_t beat = core1_heartbeat;

  Serial.printf("reports/s: %4lu | frames/s: %4lu | fails: %lu | "
                "mounts: %lu (listening: %lu) | core1 loops/s: %lu | "
                "stick: %4d %4d | buttons: %08lX | hat: %u\n",
                r - last_reports, frames_sent - last_frames, f, m, l,
                beat - last_beat, gp.z, gp.rz, gp.buttons, gp.hat);
  last_reports = r;
  last_frames = frames_sent;
  last_beat = beat;
}

void loop() {
  checkCore1();
  if (!TinyUSBDevice.mounted() || !usb_hid.ready()) return;

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < FRAME_MS) return;
  float dt = (float)(now - last);          // ms since the last frame
  last = now;

  // Take the shared state, and reset movement for the next frame.
  int32_t dx, dy;
  uint8_t mb, mods, k[6];
  mutex_enter_blocking(&shared_lock);
  dx = mouse_dx;  dy = mouse_dy;
  mouse_dx = 0;   mouse_dy = 0;
  mb = mouse_buttons;
  mods = key_mods;
  memcpy(k, keys, 6);
  mutex_exit(&shared_lock);

  // Smoothed speed: move part of the way towards this frame's speed.
  static float vx = 0, vy = 0;
  float alpha = 1.0f - expf(-dt / SMOOTH_MS);
  vx += alpha * (dx / dt - vx);
  vy += alpha * (dy / dt - vy);

  // Mouse speed -> right stick (Z / Rz in this descriptor)
  gp.z  = speedToStick(vx);
  gp.rz = speedToStick(vy);

  // IJKL -> right stick at full push (handy for Steam's setup wizard).
  // Overrides the mouse while held.
  if (keyHeld(RSTICK_LEFT, k))  gp.z  = -127;
  if (keyHeld(RSTICK_RIGHT, k)) gp.z  =  127;
  if (keyHeld(RSTICK_UP, k))    gp.rz = -127;
  if (keyHeld(RSTICK_DOWN, k))  gp.rz =  127;

  // Deadzone finder: hold P to slowly push the right stick right,
  // 0 -> 127 over ~5 s. Let go the moment the camera starts turning;
  // the last value printed is the game's deadzone.
  static int ramp = 0;
  static uint8_t ramp_tick = 0;
  if (keyHeld(DEADZONE_FINDER_KEY, k)) {
    if (++ramp_tick >= 10 && ramp < 127) {  // +1 every 10 frames (40 ms)
      ramp_tick = 0;
      ramp++;
      if (Serial && Serial.availableForWrite() > 32)
        Serial.printf("deadzone finder: %d\n", ramp);
    }
    gp.z = ramp;
  } else {
    ramp = 0;
    ramp_tick = 0;
  }

  // WASD -> left stick (up is negative)
  int x = 0, y = 0;
  if (keyHeld(LSTICK_LEFT, k))  x -= 127;
  if (keyHeld(LSTICK_RIGHT, k)) x += 127;
  if (keyHeld(LSTICK_UP, k))    y -= 127;
  if (keyHeld(LSTICK_DOWN, k))  y += 127;
  gp.x = x;
  gp.y = y;

  // Buttons, from the mapping tables
  uint32_t b = 0;
  for (auto& bind : KEY_BINDS)   if (keyHeld(bind.key, k))  b |= bind.button;
  for (auto& bind : MOD_BINDS)   if (mods & bind.mod_bit)   b |= bind.button;
  for (auto& bind : MOUSE_BINDS) if (mb & bind.mouse_bit)   b |= bind.button;
  gp.buttons = b;

  // D-pad
  gp.hat = dpadToHat(anyHeld(DPAD_UP, k),   anyHeld(DPAD_RIGHT, k),
                     anyHeld(DPAD_DOWN, k), anyHeld(DPAD_LEFT, k));

  usb_hid.sendReport(0, &gp, sizeof(gp));
  printStatus();
}

// ================= CORE 1: mouse + keyboard =================

void setup1() {
  pinMode(PIN_5V_EN, OUTPUT);       // power the USB-A port
  digitalWrite(PIN_5V_EN, HIGH);

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);
}

// Ask an interface for its next report; remember if the request failed.
void requestReport(Listener& l) {
  if (tuh_hid_receive_report(l.dev, l.inst)) {
    l.needs_retry = false;
  } else {
    l.needs_retry = true;
    mutex_enter_blocking(&shared_lock);
    request_failures++;
    mutex_exit(&shared_lock);
  }
}

Listener* findListener(uint8_t dev, uint8_t inst) {
  for (auto& l : listeners)
    if (l.active && l.dev == dev && l.inst == inst) return &l;
  return nullptr;
}

void loop1() {
  core1_heartbeat++;

  core1_stage = 1;
  USBHost.task();

  // Retry any interface whose last request failed, so the chain never dies.
  core1_stage = 2;
  for (auto& l : listeners)
    if (l.active && l.needs_retry) requestReport(l);
}

extern "C" {

void tuh_hid_mount_cb(uint8_t dev, uint8_t inst, uint8_t const*, uint16_t) {
  uint8_t proto = tuh_hid_interface_protocol(dev, inst);
  bool wanted = (proto == HID_ITF_PROTOCOL_MOUSE ||
                 proto == HID_ITF_PROTOCOL_KEYBOARD);

  // Only ask the real mouse/keyboard interfaces for data. Media keys,
  // vendor settings channels etc. are left alone, so they add no traffic.
  if (wanted) {
    for (auto& l : listeners) {
      if (!l.active) {
        l = { dev, inst, true, false };
        requestReport(l);
        break;
      }
    }
  }

  mutex_enter_blocking(&shared_lock);
  mount_count++;
  if (wanted) listen_count++;
  mutex_exit(&shared_lock);
}

void tuh_hid_umount_cb(uint8_t dev, uint8_t inst) {
  if (Listener* l = findListener(dev, inst)) {
    l->active = false;
    mutex_enter_blocking(&shared_lock);
    listen_count--;
    mutex_exit(&shared_lock);
  }
  mutex_enter_blocking(&shared_lock);
  mouse_buttons = 0;                // don't leave buttons "stuck" on unplug
  key_mods = 0;
  memset(keys, 0, 6);
  mutex_exit(&shared_lock);
}

void tuh_hid_report_received_cb(uint8_t dev, uint8_t inst,
                                uint8_t const* report, uint16_t len) {
  uint8_t proto = tuh_hid_interface_protocol(dev, inst);

  mutex_enter_blocking(&shared_lock);
  reports_received++;
  if (proto == HID_ITF_PROTOCOL_MOUSE && len >= 6) {
    // Layout worked out in the read-mouse experiments:
    // [buttons][wheel][X lo][X hi][Y lo][Y hi]
    mouse_buttons = report[0];
    mouse_dx     += read_i16(&report[2]);
    mouse_dy     += read_i16(&report[4]);
  } else if (proto == HID_ITF_PROTOCOL_KEYBOARD && len >= 8) {
    // Standard keyboard: [modifiers][reserved][6 key codes]
    key_mods = report[0];
    memcpy(keys, &report[2], 6);
  }
  mutex_exit(&shared_lock);

  if (Listener* l = findListener(dev, inst)) requestReport(*l);
}

}  // extern "C"