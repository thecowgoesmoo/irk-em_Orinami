// Orinami for Clouds hardware: pots, CV inputs and gates -> model parameters.
//
// Panel mapping (Clouds labels on the left):
//
//   POSITION pot + CV   separation: fully CCW flat, CW curls to four wraps
//   DENSITY  pot + CV   rotation, +/-180 degrees, noon = 0
//   SIZE CV             shape (added to the shape setting)
//   TEXTURE CV          mass
//   BLEND CV            excite: positive voltage lowers both dampings
//   V/OCT               both stiffnesses, 1 V/octave in frequency
//   FREEZE gate         fold kick          TRIG gate    rotation kick
//   IN L                audio              IN R         audio-rate modulator
//
//   Paged pots          RED page          GREEN page          AMBER page
//   SIZE                shape             fold kick amount    envelope -> separation
//   PITCH               fold stiffness    rot. stiffness      IN R -> separation depth
//   TEXTURE             mass              rot. kick amount    envelope -> rotation
//   BLEND               fold damping      rot. damping        spin-fold coupling
//
// A paged pot does not jump when the page changes: it keeps the stored value
// until the knob moves, then closes the gap as it travels (the scheme the
// stock Clouds firmware uses for its Blend knob).

#ifndef ORINAMI_CLOUDS_CONTROLS_H_
#define ORINAMI_CLOUDS_CONTROLS_H_

#include "stmlib/stmlib.h"

#include "clouds/drivers/adc.h"
#include "clouds/drivers/gate_input.h"

#include "orinami_clouds/dsp/core.h"
#include "orinami_clouds/settings.h"

namespace orinami_clouds {

enum Page {
  PAGE_RED,
  PAGE_GREEN,
  PAGE_AMBER
};

enum PagedPot {
  POT_SIZE,
  POT_PITCH,
  POT_TEXTURE,
  POT_BLEND
};

// Everything the audio loop needs for one block.
struct ControlFrame {
  float sep_rest;          // 1 flat .. -3
  float rot_rest;          // radians
  BlockParameters block;
  float fold_kick_amount;  // separation units
  float rot_kick_amount;   // radians
  float env_to_sep;        // separation units per unit of envelope
  float env_to_rot;        // radians per unit of envelope
  float in_r_depth;        // separation units per unit of IN R
  bool fold_kick;
  bool rot_kick;
};

class PagedKnob {
 public:
  void Init(const uint8_t* stored, int page);
  void set_page(int page);
  // Returns true if the active value changed noticeably.
  bool Update(float knob);
  inline float value(int page) const { return value_[page]; }
  inline uint8_t stored(int page) const {
    return static_cast<uint8_t>(value_[page] * 255.0f + 0.5f);
  }
  void Load(const uint8_t* stored);

 private:
  float value_[kNumPages];
  float previous_knob_;
  float origin_;
  float last_reported_;
  int page_;
};

class Controls {
 public:
  Controls() { }
  ~Controls() { }

  void Init(Settings* settings);
  void Read(ControlFrame* frame);

  void set_page(int page);
  inline int page() const { return page_; }
  inline void set_level_comp(bool on) { level_comp_ = on; }
  inline bool level_comp() const { return level_comp_; }

  // Kick request from the panel button; served on the next block.
  inline void RequestKick() { button_kick_ = true; }

  // True once after a paged pot has moved; the UI uses it to schedule a save.
  inline bool TakeDirty() {
    bool d = dirty_;
    dirty_ = false;
    return d;
  }

  void StoreTo(State* state) const;
  void LoadFrom(const State& state);

 private:
  clouds::Adc adc_;
  clouds::GateInput gate_input_;
  Settings* settings_;

  float smoothed_[clouds::ADC_CHANNEL_LAST];
  PagedKnob knob_[kNumPagedPots];
  float note_;
  int warm_up_;
  int page_;
  bool level_comp_;
  volatile bool button_kick_;
  volatile bool dirty_;

  DISALLOW_COPY_AND_ASSIGN(Controls);
};

}  // namespace orinami_clouds

#endif  // ORINAMI_CLOUDS_CONTROLS_H_
