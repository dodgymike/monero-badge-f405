# Prerequisites
You will need:
* a GCC ARM cross-compilation toolchain
* dfu-util (or equivalent)

# Converting images into header files
Convert your image to a 24x24 GIF/lossless image. Run `./graphics/convert_gif_to_colour_array.py picklerick.gif picklerick > Inc/picklerick.h`, which will give you an include file containing the `picklerick` array of pixels suitable for the display.

Add `#include "picklerick.h"` to a header file such as `Inc/welcome_smileys.h` and then point to the array in the code. Good examples (including the easter egg animation) are in `Src/welcome.c`.

# Programming the DC26 Monero badge (on Linux)
`make clean && make`

* Connect the USB port on the badge to your PC
* Power off the badge
* *HOLD THE BOOT BUTTON WHILE POWERING UP THE BADGE*

`dfu-suffix -a build/round1.bin && sudo dfu-util -a 0 -d 0483:df11 -D build/round1.bin  --dfuse-address 0x8000000`

# Contributing
Feel free to submit PRs! I will respond as soon as I am able.

# Hardware
The hardware repo can be found [here](https://github.com/dodgymike/dc26-monero-badge-pcb)

