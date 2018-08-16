# Prerequisites
You will need:
* a GCC ARM cross-compilation toolchain
* dfu-util (or equivalent)

# Programming the DC26 Monero badge (on Linux)
make clean && make && dfu-suffix -a build/round1.bin && sudo dfu-util -a 0 -d 0483:df11 -D build/round1.bin  --dfuse-address 0x8000000
