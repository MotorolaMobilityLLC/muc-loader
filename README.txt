Initial version of bootloader that handles booting the muc code.

Code here depends on the STM Cube libraries:

From the top of the repo, run
  git clone git@ocd2.mot.com:jwylder1/STM32Cube_FW_L4_V1.0.0.git


To build:

put the toolchain in your path.  For example:
    export PATH=$PATH:/usr/local/gcc-arm-none-eabi-4_8-2014q3/bin

Then run make from the top directory

Flash using:

st-flash --reset write out/boot.bin 0x08000000 
