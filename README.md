nRF51822
===================

Currently, this is only Nordic's sample ble\_hrs. And contains version 4.2 of the SDK from Nordic. In order to get it working, it is necessary to install a GCC Toolchain and JLink. This project was inspired by [this](http://www.funwithelectronics.com/?id=168) and [this](https://github.com/Sproutling/nRF51822-OSX-Sample/) project.


GCC Toolchain:
-------------------

In order to compile and link it is necessary to install a GCC toolchain. I use the one from CodeSourcery which can be downloaded [here](http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/request?id=e023fac2-e611-476b-a702-90eabb2aeca8&downloadlite=scblite2012&fmpath=/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/form). 

This link should guide you to the correct release, but make sure the the "EABI Release" is chosen. The file is distributed as a .tar.bz2 file that can be decomressed with *tar*. I moved the compiler toolchain into the /opt directory, which is reserved for all software that are not part of the default installation like so:

    tar -xvf arm-2012.09-63-arm-none-eabi-i686-pc-linux-gnu.tar.bz2
    sudo mv arm-2012.09 /opt/

In order to use the compiler with the provided Makefile, it is necessary to point the _GCC\_INSTALL\_ROOT_ variable(line 3 in Makefile) so that it points in the correct location, if you use the same compiler as I do.

JLink:
-------------------

Download the _JLink software from Segger_ from [here](http://www.segger.com/jlink-software.html?step=1&file=JLinkLinux_462a). In order to download it, it is necessary to register the serial number of the emulator that you have. This is a .tgz file that again can be decompressed with *tar*. Now that the file has been downloaded, run the following commands:

    tar -xvf JLink_Linux_V462a.tgz
    cd JLink_Linux_V462a
    sudo cp 45-jlink.rules /etc/udev/rules.d/
    cd ..
    sudo mv JLink_Linux_V462a /opt/

In order to upload programs to the MCU with JLink it is necessary to alter the _JLIINK\_DIR_(line 46 in Makefile) such that it points at the correct directory.

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
