# Prerequisites
You will need:
* a GCC ARM cross-compilation toolchain
* dfu-util (or equivalent)

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

