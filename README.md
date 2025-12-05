# Compile information

Compiled using:
```
riscv-none-elf-gcc (xPack GNU RISC-V Embedded GCC x86_64) 14.2.0
```
To compile enter example directory
```
cd examples_x033/blink
```
make

# Programming

To probram bootloader must bi activated by supplying D+ pin to VCC during startup
Then python script can be used to program:
```
sudo python chprog.py main.bin
```

![board](https://github.com/mztulip/CH32X033_examples/blob/main/board_photo2.jpeg)

![board2](https://github.com/mztulip/CH32X033_examples/blob/main/board_photo_1.jpeg)