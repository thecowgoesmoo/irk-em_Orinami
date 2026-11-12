// Orinami for Clouds hardware: buttons and LEDs.

#include "orinami_clouds/ui.h"

#include <cmath>

#include "stmlib/system/system_clock.h"

namespace orinami_clouds {

using namespace stmlib;
using namespace clouds;

const uint32_t kLongPress = 3000;      // ms
const uint32_t kSaveDelay = 2000;      // ms of quiet before writing to flash

void Ui::Init(Settings* settings, Controls* controls) {
  settings_ = settings;
  controls_ = controls;
  leds_.Init();
  switches_.Init();
  separation_ = 1.0f;
  rotation_ = 0.0f;
  load_ = 0.0f;
  kick_led_ = 0;
  for (uint8_t i = 0; i < kNumSwitches; ++i) {
    press_time_[i] = 0;
  }
  long_press_done_ = false;
  combo_ = false;
  save_pending_ = false;
  reset_pending_ = false;
  save_deadline_ = 0;
  splash_ = 600;
}

void Ui::ScheduleSave() {
  save_deadline_ = system_clock.milliseconds() + kSaveDelay;
  save_pending_ = true;
}

void Ui::Poll() {
  system_clock.Tick();
  switches_.Debounce();
  uint32_t now = system_clock.milliseconds();

  // Kick button: act on the press.
  if (switches_.just_pressed(kSwitchKick)) {
    controls_->RequestKick();
  }

  bool level_down = switches_.pressed(kSwitchLevel);
  bool page_down = switches_.pressed(kSwitchPage);
  if (level_down && page_down) {
    combo_ = true;            // load meter; neither button acts on release
  }

  // Level compensation: act on release.
  if (switches_.just_pressed(kSwitchLevel)) {
    press_time_[kSwitchLevel] = now;
  }
  if (switches_.released(kSwitchLevel) && press_time_[kSwitchLevel]) {
    if (!combo_) {
      controls_->set_level_comp(!controls_->level_comp());
      ScheduleSave();
    }
    press_time_[kSwitchLevel] = 0;
  }

  // Page: act on release; a long hold restores the defaults instead.
  if (switches_.just_pressed(kSwitchPage)) {
    press_time_[kSwitchPage] = now;
    long_press_done_ = false;
  }
  if (page_down && press_time_[kSwitchPage] && !long_press_done_ && !combo_ &&
      now - press_time_[kSwitchPage] > kLongPress) {
    long_press_done_ = true;
    reset_pending_ = true;
  }
  if (switches_.released(kSwitchPage) && press_time_[kSwitchPage]) {
    if (!combo_ && !long_press_done_) {
      controls_->set_page((controls_->page() + 1) % kNumPages);
      ScheduleSave();
    }
    press_time_[kSwitchPage] = 0;
  }
  if (!level_down && !page_down &&
      !press_time_[kSwitchLevel] && !press_time_[kSwitchPage]) {
    combo_ = false;
  }

  if (controls_->TakeDirty()) {
    ScheduleSave();
  }

  if (kick_led_) {
    --kick_led_;
  }
  if (splash_) {
    --splash_;
  }
  PaintLeds();
  leds_.Write();
}

void Ui::PaintLeds() {
  leds_.Clear();
  leds_.set_freeze(kick_led_ != 0);

  if (splash_) {
    // Power-up: the four LEDs light in turn, amber.
    uint8_t index = 3 - (splash_ / 150);
    leds_.set_status(index & 3, 255, 255);
    return;
  }

  if (combo_) {
    // Processor load as a bar: each LED is a quarter, the last one turns red
    // above 90%.
    float load = load_;
    for (uint8_t i = 0; i < 4; ++i) {
      float x = (load - 0.25f * i) * 4.0f;
      if (x <= 0.0f) continue;
      if (x > 1.0f) x = 1.0f;
      uint8_t level = static_cast<uint8_t>(x * 255.0f);
      if (i == 3 && load > 0.9f) {
        leds_.set_status(i, 255, 0);
      } else {
        leds_.set_status(i, 0, level);
      }
    }
    return;
  }

  // LED 1: curl, 0 (flat) to 4 wraps.
  float curl = (1.0f - separation_) * 0.25f;
  if (curl < 0.0f) curl = 0.0f;
  if (curl > 1.0f) curl = 1.0f;
  leds_.set_intensity(0, static_cast<uint8_t>(curl * 255.0f));

  // LED 2: rotation, green for positive angles and red for negative.
  float turn = rotation_ * (1.0f / kTwoPi);
  turn -= floorf(turn + 0.5f);              // -0.5 .. 0.5 of a turn
  float amount = fabsf(turn) * 4.0f;        // full brightness at 90 degrees
  if (amount > 1.0f) amount = 2.0f - amount;
  uint8_t level = static_cast<uint8_t>(amount * 255.0f);
  if (turn >= 0.0f) {
    leds_.set_status(1, 0, level);
  } else {
    leds_.set_status(1, level, 0);
  }

  // LED 3: level compensation.
  if (controls_->level_comp()) {
    leds_.set_status(2, 0, 255);
  }

  // LED 4: page.
  switch (controls_->page()) {
    case PAGE_RED:
      leds_.set_status(3, 255, 0);
      break;
    case PAGE_GREEN:
      leds_.set_status(3, 0, 255);
      break;
    default:
      leds_.set_status(3, 255, 255);
      break;
  }
}

void Ui::DoEvents() {
  if (reset_pending_) {
    reset_pending_ = false;
    settings_->ResetState();
    controls_->LoadFrom(settings_->state());
    splash_ = 600;
    ScheduleSave();
  }
  if (save_pending_ &&
      static_cast<int32_t>(system_clock.milliseconds() - save_deadline_) > 0) {
    save_pending_ = false;
    controls_->StoreTo(settings_->mutable_state());
    settings_->Save();
  }
}

void Ui::Panic() {
  // Codec did not start: all four LEDs red, forever.
  leds_.Init();
  for (uint8_t i = 0; i < 4; ++i) {
    leds_.set_status(i, 255, 0);
  }
  while (1) {
    leds_.Write();
  }
}

}  // namespace orinami_clouds
