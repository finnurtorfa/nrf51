nRF51822
===================

Currently, this is only Nordic's sample ble\_hrs. And contains version 4.2 of the SDK from Nordic. In order to get it working, it is necessary to install a GCC Toolchain and JLink. This project was inspired by [this](http://www.funwithelectronics.com/?id=168) and [this](https://github.com/Sproutling/nRF51822-OSX-Sample/) project.


GCC Toolchain:
-------------------

In order to compile and link it is necessary to install a GCC toolchain. I use
the ARM-EABI target provided by [Archlinux](http://www.archlinux.org) found
[here](https://www.archlinux.org/packages/community/x86_64/arm-none-eabi-gcc/)
and C standard library that goes with it. To install it on my system along with
GDB I issue the following command:

    pacman -S arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-gdb

Please refer [this site](www.google.com) for information on how to install the
GCC toolchain on you system.

JLink:
-------------------

Download the _JLink software from Segger_ from [here](http://www.segger.com/jlink-software.html?step=1&file=JLinkLinux_462a). In order to download it, it is necessary to register the serial number of the emulator that you have. This is a .tgz file that again can be decompressed with *tar*. Now that the file has been downloaded, run the following commands:

    tar -xvf JLink_Linux_V{version-number}.tgz
    cd JLink_Linux_V{version-number}
    sudo cp 99-jlink.rules /etc/udev/rules.d/
    cd ..
    mv JLink_Linux_V{version-number} ~/bin/jlink

I keep jlink under my home directory. In order to upload programs to the MCU with JLink it is necessary to alter the _JLIINK\_DIR_(line 46 in Makefile) such that it points at the correct directory.

Directory structure:
-------------------

Once you have compiled for the first time, the directory structure looks like this:

    .
    ├── bin     --> Contains the map files
    ├── build   --> Contains hex/bin/log files as well as JLink scripts
    ├── include --> Contains header files for the project
    ├── lib     --> Contains Nordics SDK as well as the softdevice hex file
    ├── obj     --> .o files
    └── src     --> Contains source files for the project

Makefile targets
-------------------

The following makefile targets exist, though not all of them have been tested

    make                    --> Compiles project.
    make all                --> Same as make
    make clean              --> Cleans all object files, output files, map files etc.
    make erase-all          --> Erases entire contents of memory
    make recover            --> Restores MCU to original state -- NOT TESTED YET
    make release            --> Same as make
    make startdebug         --> Starts GDB Server -- NOT TESTED YET
    make stop debug         --> Stops GDB Server -- NOT TESTED YET
    make upload             --> Uploads output hex file on to the MCU
    make upload-softdevice  --> Uploads the softdevice hex file on to the MCU
