// Orinami for Clouds hardware: persistent settings.

#include "orinami_clouds/settings.h"

#include "stmlib/system/storage.h"

namespace orinami_clouds {

const uint8_t kStateVersion = 1;

// Default positions of the paged pots, 0..255. Row = pot, column = page
// (red, green, amber). See controls.cc for what each one means.
const uint8_t kDefaultPots[kNumPagedPots * kNumPages] = {
  // SIZE:    shape (circle), fold kick (+0.3), envelope -> separation (off)
  128, 151, 128,
  // PITCH:   fold stiffness (1e3), rotation stiffness (1e2), In R depth (off)
  113, 85, 128,
  // TEXTURE: mass (1), rotation kick (+30 deg), envelope -> rotation (off)
  128, 153, 128,
  // BLEND:   fold damping (0.05), rotation damping (0.05), coupling (100%)
  148, 148, 255
};

void Settings::ResetState() {
  state_.version = kStateVersion;
  state_.page = 0;
  state_.level_comp = 0;
  state_.padding = 0;
  for (int i = 0; i < kNumPagedPots * kNumPages; ++i) {
    state_.pot[i] = kDefaultPots[i];
  }
}

void Settings::Init() {
  // Stock Clouds calibration, read-only.
  uint16_t clouds_token;
  calibrated_ = stmlib::Storage<1>::ParsimoniousLoad(&clouds_, &clouds_token);
  if (!calibrated_) {
    clouds_.calibration_data.pitch_offset = 66.67f;
    clouds_.calibration_data.pitch_scale = -84.26f;
    for (size_t i = 0; i < clouds::ADC_CHANNEL_LAST; ++i) {
      clouds_.calibration_data.offset[i] = 0.505f;
    }
  }

  // Orinami's own state. Nothing is written until the first change.
  if (!stmlib::Storage<7>::ParsimoniousLoad(&state_, &version_token_) ||
      state_.version != kStateVersion ||
      state_.page >= kNumPages) {
    ResetState();
  }
}

void Settings::Save() {
  stmlib::Storage<7>::ParsimoniousSave(state_, &version_token_);
}

}  // namespace orinami_clouds
