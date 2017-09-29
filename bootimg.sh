#!/bin/bash
mkdir out/out1
../out/host/linux-86/bin/mkimage out/arch/arm/boot/zImage KERNEL 0xffffffff > out/kernel.bin
cp out/kernel.bin out/out1/kernel

../out/host/linux-86/bin/mkbootimg --kernel out/out1/kernel --ramdisk out/ramdisk.img --base 0x80000000 --ramdisk_offset 0x04000000 --kernel_offset 0x00008000 --tags_offset 0x0000100 --board 1432786960 --output out/boot.img
