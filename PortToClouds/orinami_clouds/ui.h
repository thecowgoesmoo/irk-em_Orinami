// Orinami for Clouds hardware: buttons and LEDs.
//
//   FREEZE button        kick both fold and rotation (its LED flashes on any kick)
//   Left small button    level compensation on/off        (SW_MODE on the schematic)
//   Right small button   next page: red -> green -> amber  (SW_LOAD on the schematic)
//     held 3 s           restore the default settings
//   Both small buttons   while held, the LEDs show processor load
//
//   LED 1   fold: dark when flat, green -> amber -> red as the screen curls
//   LED 2   rotation: green one way, red the other
//   LED 3   green when level compensation is on
//   LED 4   page colour
//
// To swap the two small buttons, exchange kSwitchLevel and kSwitchPage.

#ifndef ORINAMI_CLOUDS_UI_H_
#define ORINAMI_CLOUDS_UI_H_

#include "stmlib/stmlib.h"

#include "clouds/drivers/leds.h"
#include "clouds/drivers/switches.h"

#include "orinami_clouds/controls.h"
#include "orinami_clouds/settings.h"

namespace orinami_clouds {

const uint8_t kSwitchLevel = 0;    // SW_MODE
const uint8_t kSwitchPage = 1;     // SW_LOAD
const uint8_t kSwitchKick = 2;     // FREEZE

class Ui {
 public:
  Ui() { }
  ~Ui() { }

  void Init(Settings* settings, Controls* controls);
  void Poll();       // 1 kHz, from the SysTick interrupt
  void DoEvents();   // main loop: flash writes happen here
  void Panic();

  // Called from the audio interrupt.
  inline void set_state(float separation, float rotation) {
    separation_ = separation;
    rotation_ = rotation;
  }
  inline void NotifyKick() { kick_led_ = 60; }
  inline void set_load(float load) {
    if (load > load_) load_ = load; else load_ += 0.01f * (load - load_);
  }

 private:
  void PaintLeds();
  void ScheduleSave();

  Settings* settings_;
  Controls* controls_;
  clouds::Leds leds_;
  clouds::Switches switches_;

  volatile float separation_;
  volatile float rotation_;
  volatile float load_;
  volatile uint16_t kick_led_;

  uint32_t press_time_[clouds::kNumSwitches];
  bool long_press_done_;
  bool combo_;
  volatile bool save_pending_;
  volatile bool reset_pending_;
  volatile uint32_t save_deadline_;
  uint16_t splash_;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace orinami_clouds

#endif  // ORINAMI_CLOUDS_UI_H_
