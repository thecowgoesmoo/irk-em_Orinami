// Orinami for Clouds hardware: persistent settings.
//
// Two flash areas are involved:
//  - Sector 1 holds the stock Clouds settings, including the factory
//    calibration (CV offsets, V/oct scale). Orinami READS it and never
//    writes it, so reflashing Clouds later finds its calibration intact.
//  - Sector 7 (0x08060000) holds Orinami's own state. Stock Clouds and
//    Orinami are both far smaller than the 352 kB below that address.

#ifndef ORINAMI_CLOUDS_SETTINGS_H_
#define ORINAMI_CLOUDS_SETTINGS_H_

#include "stmlib/stmlib.h"

#include "clouds/drivers/adc.h"

namespace orinami_clouds {

const int kNumPages = 3;
const int kNumPagedPots = 4;

// Layout of the stock Clouds settings block (clouds/settings.h).
struct CloudsCalibrationData {
  float pitch_offset;
  float pitch_scale;
  float offset[clouds::ADC_CHANNEL_LAST];
};

struct CloudsSettingsData {
  CloudsCalibrationData calibration_data;   // 48 bytes
  uint8_t state[8];
  uint8_t padding[8];
};

struct State {
  uint8_t version;
  uint8_t page;
  uint8_t level_comp;
  uint8_t padding;
  uint8_t pot[kNumPagedPots * kNumPages];   // 0..255
};

class Settings {
 public:
  Settings() { }
  ~Settings() { }

  void Init();
  void Save();
  void ResetState();

  inline const CloudsCalibrationData& calibration_data() const {
    return clouds_.calibration_data;
  }
  inline bool calibrated() const { return calibrated_; }
  inline State* mutable_state() { return &state_; }
  inline const State& state() const { return state_; }

 private:
  CloudsSettingsData clouds_;
  State state_;
  bool calibrated_;
  uint16_t version_token_;

  DISALLOW_COPY_AND_ASSIGN(Settings);
};

}  // namespace orinami_clouds

#endif  // ORINAMI_CLOUDS_SETTINGS_H_
