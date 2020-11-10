# Edge Impulse firmware for ST B-L475E-IOT01A

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the ST B-L475E-IOT01A development board. This device supports all Edge Impulse device features, including ingestion, remote management and inferencing.

## Requirements

**Hardware**

* [DISCO-L475VG-IOT01A](https://os.mbed.com/platforms/ST-Discovery-L475E-IOT01A/) development board.

**Software**

* [Node.js 10.16](https://nodejs.org/en/download/) or higher.
* [Git](https://git-scm.com/downloads) - make sure `git` is in your PATH.
* [Mercurial](https://www.mercurial-scm.org) - make sure `hg` is in your PATH.
* [GNU ARM Embedded Toolchain 9-2019-q4-major](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).
* STLink tools.

    Installation via Homebrew is the easiest:

    ```
    $ brew install stlink
    ```

* [Mbed CLI](https://github.com/ARMmbed/mbed-cli).

    Installation via pip is the easiest:

    ```
    $ pip install mbed-cli
    ```

Unpack the GNU ARM Embedded Toolchain, and configure Mbed CLI to use it via:

```
$ mbed config -G GCC_ARM_PATH ~/toolchains/gcc-arm-none-eabi-9-2019-q4-major/bin/
$ mbed config -G TOOLCHAIN GCC_ARM
$ mbed config -G PROTOCOL SSH
```

## Building the device firmware

1. Clone this repository:

    ```
    $ git clone https://github.com/edgeimpulse/firmware-st-b-l475e-iot01a
    ```

1. Update dependencies:

    ```
    $ mbed deploy
    ```

1. Fix an outdated file in the `mbed-os` dependency:

    ```
    cp source/edge-impulse-sdk/CMSIS/Core/Include/cmsis_gcc.h mbed-os/cmsis/TARGET_CORTEX_M/cmsis_gcc.h
    ```

1. Build and flash this project:

    ```
    $ mbed compile -t GCC_ARM -m DISCO_L475VG_IOT01A --profile=debug -f
    ```

1. Attach a serial monitor to the board on baud rate 115,200 to see the output.

    On macOS you can use [Serial.app](https://www.decisivetactics.com/products/serial/) (recommended!) or connect via `screen`:

    1. Find the handle for your board:

        ```
        $ ls /dev/tty.usbm*
        /dev/tty.usbmodem401203
        ```

    1. Then connect via:

        ```
        $ screen /dev/tty.usbmodem401203 115200
        ```

    1. To exit, press: `CTRL+A` then `CTRL+\` then press `y`.

## Debugging through Visual Studio Code

1. Install STLink:

    ```
    $ brew install stlink
    ```

1. Install mbed-vscode-generator:

    ```
    $ npm install mbed-vscode-generator -g
    ```

1. Generate the debugger files (run from the root folder of this project, not from the firmware folder):

    ```
    $ mbed-vscode-generator -i firmware/ -o .vscode/ --debugger stlink
    ```

1. Just press 'Run' in Visual Studio Code to build and debug.

If the debugger does not properly detach, run `killall st-util`.

## Updating the TensorFlow Lite for Microcontrollers library

To update to a different commit of the TensorFlow Lite for Microcontrollers library, edit the script `update_tflite.sh` and include the new commit hash. Run the script from the parent directory of the firmware.

This script also ensures the CMSIS libraries are the same version used by the TensorFlow Lite project.
