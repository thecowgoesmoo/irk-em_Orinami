// Orinami for Clouds hardware.
//
// A waveshaper modelled on a foldable, spinnable oscilloscope screen, running
// on Mutable Instruments Clouds (STM32F405 + WM8731) in place of the stock
// firmware. Uses the Clouds drivers and bootloader unchanged.
//
// Audio path, per 16-sample block at 48 kHz:
//   controls.Read()     pots, CVs, gates -> model parameters (3 kHz)
//   core.Step()         fold and rotation dynamics, every sample
//   core.Project() x2   the screen mapping, 2x oversampled
//
// Based on the Clouds firmware by Emilie Gillet (MIT licence).

#include <stm32f4xx_conf.h>

#include "clouds/drivers/codec.h"
#include "clouds/drivers/system.h"
#include "clouds/drivers/version.h"

#include "orinami_clouds/controls.h"
#include "orinami_clouds/dsp/core.h"
#include "orinami_clouds/dsp/oversampler.h"
#include "orinami_clouds/settings.h"
#include "orinami_clouds/ui.h"

using namespace clouds;
using namespace orinami_clouds;

const int32_t kSampleRate = 48000;
const size_t kBlockSize = 16;

// The screen spans s = -1..+1. Clouds passes a +/-5 V signal at about half of
// codec full scale with IN GAIN at its usual setting, so the input is doubled
// to reach the ends of the screen and the output halved to keep a flat screen
// at unity gain.
const float kInputScale = 2.0f / 32768.0f;
const float kOutputScale = 0.5f * 32767.0f;

// Envelope follower on the audio input: fast attack, about 150 ms release.
const float kEnvAttack = 0.02f;
const float kEnvRelease = 0.00014f;

Codec codec;
Controls controls;
Settings settings;
Ui ui;

Tables tables;
Core core;
Upsampler2x upsampler;
Downsampler2x downsampler_def;
Downsampler2x downsampler_orth;

ControlFrame frame;
float sep_rest = 1.0f;
float rot_rest = 0.0f;
float envelope = 0.0f;

int __errno;

// Cortex-M4 cycle counter, used for the processor load display.
#define DEMCR      (*(volatile uint32_t*)0xE000EDFC)
#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t*)0xE0001004)

extern "C" {

void NMI_Handler() { }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler() { while (1); }
void UsageFault_Handler() { while (1); }
void SVC_Handler() { }
void DebugMon_Handler() { }
void PendSV_Handler() { }

void SysTick_Handler() {
  ui.Poll();
}

}

inline short Clip16(float x) {
  if (x > 32767.0f) x = 32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return static_cast<short>(x);
}

void FillBuffer(Codec::Frame* input, Codec::Frame* output, size_t n) {
  uint32_t start = DWT_CYCCNT;

  controls.Read(&frame);
  core.Prepare(frame.block);
  if (frame.fold_kick) {
    core.KickFold(frame.fold_kick_amount);
  }
  if (frame.rot_kick) {
    core.KickRotation(frame.rot_kick_amount);
  }
  if (frame.fold_kick || frame.rot_kick) {
    ui.NotifyKick();
  }

  // Ramp the rest positions across the block so CV steps do not zipper.
  float step = 1.0f / static_cast<float>(n);
  float sep_increment = (frame.sep_rest - sep_rest) * step;
  float rot_increment = (frame.rot_rest - rot_rest) * step;

  for (size_t i = 0; i < n; ++i) {
    float x = static_cast<float>(input[i].l) * kInputScale;
    float modulator = static_cast<float>(input[i].r) * kInputScale;

    float level = x < 0.0f ? -x : x;
    envelope += (level > envelope ? kEnvAttack : kEnvRelease) *
        (level - envelope);

    sep_rest += sep_increment;
    rot_rest += rot_increment;
    float sep = sep_rest
        - frame.env_to_sep * envelope
        - frame.in_r_depth * modulator;
    if (sep > 1.0f) sep = 1.0f;
    if (sep < kSepMin) sep = kSepMin;
    float rot = rot_rest + frame.env_to_rot * envelope;

    core.Step(sep, rot);

    float xs[2], d[2], o[2];
    upsampler.Process(x, xs);
    core.Project(xs[0], 0.5f, &d[0], &o[0]);
    core.Project(xs[1], 1.0f, &d[1], &o[1]);
    output[i].l = Clip16(downsampler_def.Process(d) * kOutputScale);
    output[i].r = Clip16(downsampler_orth.Process(o) * kOutputScale);
  }

  ui.set_state(core.separation(), core.rotation());
  uint32_t elapsed = DWT_CYCCNT - start;
  ui.set_load(static_cast<float>(elapsed) /
      (static_cast<float>(n) * (F_CPU / kSampleRate)));
}

void Init() {
  System sys;
  Version version;

  sys.Init(true);
  version.Init();

  DEMCR |= 0x01000000;      // enable the cycle counter
  DWT_CYCCNT = 0;
  DWT_CTRL |= 1;

  tables.Init();
  core.Init(&tables, static_cast<float>(kSampleRate));
  upsampler.Init();
  downsampler_def.Init();
  downsampler_orth.Init();

  settings.Init();
  controls.Init(&settings);
  ui.Init(&settings, &controls);

  // Zero-initialise the frame in case the codec fires before the first read.
  frame.sep_rest = 1.0f;
  frame.rot_rest = 0.0f;

  bool master = !version.revised();
  if (!codec.Init(master, kSampleRate)) {
    ui.Panic();
  }
  if (!codec.Start(kBlockSize, &FillBuffer)) {
    ui.Panic();
  }
  sys.StartTimers();
}

int main(void) {
  Init();
  while (1) {
    ui.DoEvents();
  }
}
