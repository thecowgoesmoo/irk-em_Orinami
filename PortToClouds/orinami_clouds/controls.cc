// Orinami for Clouds hardware: pots, CV inputs and gates -> model parameters.

#include "orinami_clouds/controls.h"

#include <algorithm>
#include <cmath>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/units.h"

namespace orinami_clouds {

using namespace std;
using namespace clouds;

// ---- Tuning constants ------------------------------------------------------

// One-pole smoothing per ADC channel, applied once per audio block (3 kHz at
// 48 kHz / 16 samples). 0.5 passes CV up to about 330 Hz; 0.02 is a steady
// pot. Raise the CV values for faster modulation, lower them if a CV input
// sounds noisy.
struct Channel {
  bool flip;
  bool remove_offset;
  float coefficient;
};

const Channel kChannels[ADC_CHANNEL_LAST] = {
  { true, false, 0.5f },    // ADC_POSITION_POTENTIOMETER_CV  separation
  { true, false, 0.5f },    // ADC_DENSITY_POTENTIOMETER_CV   rotation
  { false, false, 0.02f },  // ADC_SIZE_POTENTIOMETER
  { false, true, 0.5f },    // ADC_SIZE_CV                    shape
  { false, false, 0.02f },  // ADC_PITCH_POTENTIOMETER
  { false, false, 1.0f },   // ADC_V_OCT_CV                   stiffness
  { false, false, 0.02f },  // ADC_BLEND_POTENTIOMETER
  { false, true, 0.5f },    // ADC_BLEND_CV                   excite
  { false, false, 0.02f },  // ADC_TEXTURE_POTENTIOMETER
  { false, true, 0.2f }     // ADC_TEXTURE_CV                 mass
};

const float kStiffnessCentre = 3162.0f;   // stiffness with the pot at noon
const float kStiffnessSemitones = 89.7f;  // pot half-travel: x10^4.5 in k
const float kMassSemitones = 79.73f;      // pot half-travel: x100 in mass
const float kMaxFoldKick = 2.0f;          // separation units at full travel
const float kMaxRotKick = kPi;            // radians at full travel
const float kMaxEnvToSep = 4.0f;          // separation units per unit envelope
const float kMaxEnvToRot = kPi;           // radians per unit envelope
const float kMaxInRDepth = 2.0f;          // separation units per unit of IN R
const float kExcitePerUnit = 1.0f;        // damping ratio per CV unit (5 V = 0.5)

// Pot 0..1 -> -1..+1 with a flat spot at noon so "zero" is easy to find.
inline float Bipolar(float p) {
  float x = 2.0f * p - 1.0f;
  const float dead = 0.04f;
  if (x > dead) return (x - dead) / (1.0f - dead);
  if (x < -dead) return (x + dead) / (1.0f - dead);
  return 0.0f;
}

// Damping ratio from -1..+1: right of noon 0..2 (squared, fine near zero),
// left of noon 0..-0.2 (self-oscillation).
inline float DampingRatio(float x) {
  return x >= 0.0f ? 2.0f * x * x : 0.2f * x;
}

// ---- PagedKnob --------------------------------------------------------------

void PagedKnob::Load(const uint8_t* stored) {
  for (int i = 0; i < kNumPages; ++i) {
    value_[i] = static_cast<float>(stored[i]) / 255.0f;
  }
}

void PagedKnob::Init(const uint8_t* stored, int page) {
  Load(stored);
  page_ = page;
  previous_knob_ = -1.0f;   // filled in by the first reading
  origin_ = -1.0f;
  last_reported_ = value_[page];
}

void PagedKnob::set_page(int page) {
  page_ = page;
  origin_ = previous_knob_;  // ignore jitter until the knob really moves
  last_reported_ = value_[page];
}

bool PagedKnob::Update(float knob) {
  if (previous_knob_ == -1.0f) {
    // First reading after power-up: keep the stored value, exactly as after
    // a page change, so a patch comes back as it was left.
    previous_knob_ = knob;
    origin_ = knob;
  }
  float value = value_[page_];
  float delta = knob - previous_knob_;
  // Scale the travel so that the value reaches the end of its range exactly
  // when the knob reaches the end of its rotation.
  float skew = delta > 0.0f
      ? (1.001f - value) / (1.001f - previous_knob_)
      : (0.001f + value) / (0.001f + previous_knob_);
  CONSTRAIN(skew, 0.1f, 10.0f);
  if (origin_ >= 0.0f && fabsf(knob - origin_) < 0.02f) {
    delta = 0.0f;
  } else {
    origin_ = -1.0f;
  }
  value += skew * delta;
  CONSTRAIN(value, 0.0f, 1.0f);
  value_[page_] = value;
  previous_knob_ = knob;

  if (fabsf(value - last_reported_) > 0.004f) {
    last_reported_ = value;
    return true;
  }
  return false;
}

// ---- Controls ---------------------------------------------------------------

void Controls::Init(Settings* settings) {
  settings_ = settings;
  adc_.Init();
  gate_input_.Init();
  fill(&smoothed_[0], &smoothed_[ADC_CHANNEL_LAST], 0.0f);
  note_ = 0.0f;
  warm_up_ = 32;
  button_kick_ = false;
  dirty_ = false;
  LoadFrom(settings->state());
}

void Controls::LoadFrom(const State& state) {
  page_ = state.page;
  level_comp_ = state.level_comp != 0;
  for (int i = 0; i < kNumPagedPots; ++i) {
    knob_[i].Init(&state.pot[i * kNumPages], page_);
  }
}

void Controls::StoreTo(State* state) const {
  state->page = page_;
  state->level_comp = level_comp_ ? 1 : 0;
  for (int i = 0; i < kNumPagedPots; ++i) {
    for (int j = 0; j < kNumPages; ++j) {
      state->pot[i * kNumPages + j] = knob_[i].stored(j);
    }
  }
}

void Controls::set_page(int page) {
  page_ = page;
  for (int i = 0; i < kNumPagedPots; ++i) {
    knob_[i].set_page(page);
  }
}

void Controls::Read(ControlFrame* frame) {
  const CloudsCalibrationData& calibration = settings_->calibration_data();

  for (size_t i = 0; i < ADC_CHANNEL_LAST; ++i) {
    const Channel& channel = kChannels[i];
    float value = adc_.float_value(i);
    if (channel.flip) {
      value = 1.0f - value;
    }
    if (channel.remove_offset) {
      value -= calibration.offset[i];
    }
    if (warm_up_) {
      smoothed_[i] = value;   // let the converter settle before filtering
    } else {
      smoothed_[i] += channel.coefficient * (value - smoothed_[i]);
    }
  }

  // Paged pots.
  if (warm_up_) {
    --warm_up_;
  } else {
    bool moved = false;
    moved |= knob_[POT_SIZE].Update(smoothed_[ADC_SIZE_POTENTIOMETER]);
    moved |= knob_[POT_PITCH].Update(smoothed_[ADC_PITCH_POTENTIOMETER]);
    moved |= knob_[POT_TEXTURE].Update(smoothed_[ADC_TEXTURE_POTENTIOMETER]);
    moved |= knob_[POT_BLEND].Update(smoothed_[ADC_BLEND_POTENTIOMETER]);
    if (moved) {
      dirty_ = true;
    }
  }

  // The CV inputs are inverting: a positive voltage reads below the offset.
  // After offset removal +/-5 V is -/+0.5.
  float cv_shape = -2.0f * smoothed_[ADC_SIZE_CV];      // +/-5 V -> +/-1
  float cv_mass = -2.0f * smoothed_[ADC_TEXTURE_CV];    // +/-5 V -> +/-1
  float cv_excite = -smoothed_[ADC_BLEND_CV];           // +/-5 V -> +/-0.5

  // Separation: pot and CV are already summed in hardware.
  float position = (smoothed_[ADC_POSITION_POTENTIOMETER_CV] - 0.02f) / 0.96f;
  CONSTRAIN(position, 0.0f, 1.0f);
  frame->sep_rest = 1.0f - 4.0f * position;

  // Rotation.
  float density = smoothed_[ADC_DENSITY_POTENTIOMETER_CV];
  CONSTRAIN(density, 0.0f, 1.0f);
  frame->rot_rest = kPi * Bipolar(density);

  // Shape.
  float shape = Bipolar(knob_[POT_SIZE].value(PAGE_RED)) + cv_shape;
  CONSTRAIN(shape, -1.0f, 1.0f);
  frame->block.shape_index = ShapeIndex(shape);

  // V/oct, with the stock Clouds calibration: 0 V = 0, 12 per volt.
  float note = calibration.pitch_offset +
      smoothed_[ADC_V_OCT_CV] * calibration.pitch_scale;
  if (fabsf(note - note_) > 0.5f) {
    note_ = note;
  } else {
    ONE_POLE(note_, note, 0.2f)
  }

  // Stiffness: pot sets k over nine decades, V/oct scales both by 4x per volt
  // so each axis's natural frequency tracks 1 V/octave.
  float fold_semitones = kStiffnessSemitones *
      (2.0f * knob_[POT_PITCH].value(PAGE_RED) - 1.0f) + note_;
  float rot_semitones = kStiffnessSemitones *
      (2.0f * knob_[POT_PITCH].value(PAGE_GREEN) - 1.0f) + note_;
  CONSTRAIN(fold_semitones, -120.0f, 120.0f);
  CONSTRAIN(rot_semitones, -120.0f, 120.0f);
  float fold_ratio = stmlib::SemitonesToRatio(fold_semitones);
  float rot_ratio = stmlib::SemitonesToRatio(rot_semitones);
  frame->block.kf = kStiffnessCentre * fold_ratio * fold_ratio;
  frame->block.kr = kStiffnessCentre * rot_ratio * rot_ratio;

  // Mass.
  float mass = 2.0f * knob_[POT_TEXTURE].value(PAGE_RED) - 1.0f + cv_mass;
  CONSTRAIN(mass, -1.0f, 1.0f);
  frame->block.m = stmlib::SemitonesToRatio(kMassSemitones * mass);

  // Damping. The excite CV lowers both, so a gate can start an oscillation.
  float zf = DampingRatio(2.0f * knob_[POT_BLEND].value(PAGE_RED) - 1.0f);
  float zr = DampingRatio(2.0f * knob_[POT_BLEND].value(PAGE_GREEN) - 1.0f);
  zf -= kExcitePerUnit * cv_excite;
  zr -= kExcitePerUnit * cv_excite;
  CONSTRAIN(zf, -0.2f, 2.0f);
  CONSTRAIN(zr, -0.2f, 2.0f);
  frame->block.zf = zf;
  frame->block.zr = zr;

  frame->block.couple = knob_[POT_BLEND].value(PAGE_AMBER);
  frame->block.level_comp = level_comp_;

  frame->fold_kick_amount =
      kMaxFoldKick * Bipolar(knob_[POT_SIZE].value(PAGE_GREEN));
  frame->rot_kick_amount =
      kMaxRotKick * Bipolar(knob_[POT_TEXTURE].value(PAGE_GREEN));
  frame->env_to_sep = kMaxEnvToSep * Bipolar(knob_[POT_SIZE].value(PAGE_AMBER));
  frame->env_to_rot =
      kMaxEnvToRot * Bipolar(knob_[POT_TEXTURE].value(PAGE_AMBER));
  frame->in_r_depth =
      kMaxInRDepth * Bipolar(knob_[POT_PITCH].value(PAGE_AMBER));

  // Gates: FREEZE kicks the fold, TRIG kicks the rotation, the button both.
  gate_input_.Read();
  bool button = button_kick_;
  button_kick_ = false;
  frame->fold_kick = gate_input_.freeze_rising_edge() || button;
  frame->rot_kick = gate_input_.trigger_rising_edge() || button;

  adc_.Convert();
}

}  // namespace orinami_clouds
