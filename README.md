# Orinami

VCV Rack 2 module: a waveshaper modelled on a foldable, spinnable oscilloscope
screen whose fold and rotation are sprung, coupled second-order systems.

## Build

1. Install the Rack plugin toolchain for your OS:
   https://vcvrack.com/manual/Building#Setting-up-your-development-environment
2. Download and unzip the Rack SDK (2.x) from https://vcvrack.com/downloads/
3. From this folder:

       export RACK_DIR=/path/to/Rack-SDK
       make install

   `make install` builds the plugin and copies it into your Rack user folder.
   Restart Rack and look for "Orinami" in the module browser.

## Files

- `src/OrinamiCore.hpp`  DSP core, no Rack dependencies (reusable for Teensy)
- `src/Orinami.cpp`      Rack module and panel widget
- `res/Orinami.svg`      Panel background; labels are drawn in code

## Controls

Each large knob is an attenuverter on the CV jack below it. An unpatched jack
reads +10 V, so the knob alone sets the value; value = knob x CV / 10 V.

| Control | Range |
|---|---|
| SEP | noon = flat; either direction curls, up to 4 wraps at the ends |
| ROT | +/-180 degrees, noon = 0 |
| SHAPE | L1 pleats (left), circle (noon), square corners (right) |
| FOLD STIFF / ROTATION STIFF | 0.1 to 1e8, logarithmic |
| FOLD MASS | 0.01 to 100, logarithmic; also sets the rotational inertia |
| FOLD DAMP / ROTATION DAMP | right of noon: ratio 0 to 2; left of noon: 0 to -0.2 (self-oscillation) |
| COUPLE | how strongly the spin throws the fold masses outward |
| AMOUNT, KICK, TRIG | size of a kick; fire it with the button or a trigger |
| DC BLK, LEVEL | optional output DC blocker and fold level compensation |

Audio in and out are +/-5 V. Oversampling (1x to 8x) is in the right-click menu.
