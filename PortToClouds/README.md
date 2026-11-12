# Orinami for Clouds

Alternative firmware that turns a Mutable Instruments Clouds (or a hardware-compatible
clone) into Orinami: a waveshaper modelled on a foldable, spinnable oscilloscope screen
whose fold and rotation are sprung, coupled second-order systems.

Status: first build. It compiles, the DSP and control mapping are tested on a desktop, and
the update WAV decodes correctly through the bootloader's own decoder. It has not yet run
on hardware.

## Files

| File | What it is |
|---|---|
| `orinami_clouds.wav` | Firmware update for the audio bootloader |
| `orinami_clouds.bin` | The same firmware as a raw binary (load at 0x08008000 with a programmer) |
| `orinami_clouds/` | Source; builds inside the Mutable Instruments `eurorack` repository |

## Installing

Your module's bootloader, factory calibration and saved Clouds buffers are not touched.

1. Unplug every cable from the module. Connect your computer or phone's audio output to **IN L**.
2. Power the module on while holding the **FREEZE** button. Its LED blinks: the
   bootloader is waiting.
3. Play `orinami_clouds.wav` (about 40 seconds) at a healthy level, with no system sounds
   or EQ. The LEDs work as a level meter; raise the level or **IN GAIN** until several are lit.
4. All four LEDs go green while each block is written. When the file ends the module
   restarts as Orinami: the four LEDs light in turn, amber.
5. If the LEDs blink red, the signal was not understood. Press FREEZE, adjust the level,
   and play the file again from the start.

To go back, repeat the procedure with the stock Clouds firmware WAV from the Mutable
Instruments documentation site.

## Panel

Always live:

| Clouds label | Orinami function |
|---|---|
| POSITION knob + CV | Separation: fully counter-clockwise is flat, clockwise curls up to four wraps |
| DENSITY knob + CV | Rotation, ±180°, noon = 0° |
| IN GAIN | Input level (analog) |
| SIZE CV | Shape (adds to the Shape setting) |
| TEXTURE CV | Mass |
| BLEND CV | Excite: positive voltage lowers both dampings (5 V = 0.5), so a gate can start an oscillation |
| V/OCT | Both stiffnesses, tracking 1 V/octave in frequency |
| FREEZE gate | Kick the fold |
| TRIG gate | Kick the rotation |
| IN L | Audio |
| IN R | Audio-rate modulator for separation (amount on the amber page) |
| OUT L / OUT R | Default / orthogonal projection |

Paged knobs. The right-hand small button steps through the pages; LED 4 shows the colour.

| Knob | Red | Green | Amber |
|---|---|---|---|
| SIZE | Shape: pleats, arc (noon), square | Fold kick amount (noon = none) | Input envelope to separation (noon = none) |
| PITCH | Fold stiffness | Rotation stiffness | IN R to separation depth (noon = none) |
| TEXTURE | Mass | Rotation kick amount (noon = none) | Input envelope to rotation (noon = none) |
| BLEND | Fold damping | Rotation damping | Spin-fold coupling |

Damping knobs: right of noon is ordinary damping (0 to 2); left of noon is negative, and
that axis oscillates by itself.

A paged knob does not jump when you change page. It keeps its stored value until you turn
it, then closes the gap as it travels, as the stock Blend knob does.

Buttons and LEDs:

| Control | Function |
|---|---|
| FREEZE button | Kick fold and rotation together (its LED flashes on any kick) |
| Left small button | Level compensation on/off |
| Right small button | Next page; hold 3 s to restore the default settings |
| Both small buttons held | LEDs show processor load, a quarter per LED; the last turns red above 90% |
| LED 1 | Fold: dark when flat, green to amber to red as the screen curls |
| LED 2 | Rotation: green one way, red the other |
| LED 3 | Green when level compensation is on |
| LED 4 | Page colour |

Settings (page, level compensation, all twelve paged values) are saved two seconds after
the last change and restored at power-up.

## Things to check on the first run

These depend on hardware I could not test:

- **Which small button is which.** If page and level compensation are swapped, exchange
  `kSwitchLevel` and `kSwitchPage` in `ui.h`.
- **Signal level.** `kInputScale` and `kOutputScale` in `orinami_clouds.cc` assume a ±5 V
  input sits at about half of converter full scale at the usual IN GAIN setting.
- **Processor load.** Hold both small buttons. If it is high, the first thing to drop is
  2x oversampling.
- **CV noise and speed.** The smoothing per input is the table at the top of `controls.cc`.
  CVs are read 3,000 times a second.
- **First save.** The first time a setting is saved the module may pause for a second or
  two while it prepares its flash sector. Later saves are instant.

## Building

Needs the `arm-none-eabi-gcc` toolchain and Python 3 with numpy.

    git clone --recursive https://github.com/pichenettes/eurorack.git
    cp -r orinami_clouds eurorack/
    cd eurorack
    make -f orinami_clouds/makefile TOOLCHAIN_PATH=/path/to/toolchain/ bin
    make -f orinami_clouds/makefile TOOLCHAIN_PATH=/path/to/toolchain/ wav

`TOOLCHAIN_PATH` is the folder that contains `bin/arm-none-eabi-gcc`, with a trailing slash.
Output lands in `build/orinami_clouds/`.

## Source layout

| File | Purpose |
|---|---|
| `orinami_clouds.cc` | Start-up and the audio callback (48 kHz, 16-sample blocks) |
| `dsp/core.h` | The model: screen geometry tables, fold and rotation dynamics, projection |
| `dsp/oversampler.h` | 2x halfband up/down sampling |
| `controls.cc/.h` | Pots, CVs and gates to model parameters; paged knobs |
| `ui.cc/.h` | Buttons and LEDs |
| `settings.cc/.h` | Reads the stock calibration (flash sector 1, never written); stores Orinami state in sector 7 |
| `tools/encoder.py` | Firmware .bin to bootloader .wav (Python 3 port of the Mutable Instruments tool) |
| `tools/boot_verify.cc` | Desktop check: decodes a .wav with the bootloader's own code and compares it with the .bin |

The drivers (`clouds/drivers`) and bootloader are the stock Mutable Instruments ones.

## Licence and credit

The Clouds drivers, bootloader, build system and the code this firmware is adapted from are
copyright Emilie Gillet, released under the MIT licence. "Mutable Instruments" and "Clouds"
are names of the original maker; this firmware is not affiliated with or endorsed by them.
