# Bootloader

## What is a bootloader?
A bootloader is a small piece of software that **is reponsible for loading and initializing the main application program** on an microcontroller or other embedded device.

The bootloader is usually stored in a special area of memory thta is reserved for boot code, and it runs inmediately after the devices is powered on or reset.

## What is the need of a bootloader?
The main purpose of a bootloader is to provide a way to update the firmware on the device without requiring specialized hardware or tools.

This specially useful for devices thta are deployed in the fieldm where it may not be practical or cost-effective to physically connect to the device and program it using traditional methods.

With a Bootloader, the firmware can be updated over the air, wire or wireless connection, or even by the end-user themselves using a simple interface.

## Main uses of a bootloader.
**Firmware updates:**
A bootloader allows for easy and convenient firmware updates to the device, without requiring specialized hardware or tools.

**Recovery mode:**
In case the main firmware becomes corrupted or fails, a bootloader can provide a recovery mode that allows for the device to be restored to a working state. This is specially useful for critical applications where downtime can be costly.

Security Bootloaders can provide an added layer of security by allowing for firmware updates to be perform only by authorized personnel or devices. This can help to prevent unauthorized access or tampering with the device.

## Drivers.
### FPU
The FPU fully supports single-precision add, subtract, multiply, divide, multiply and accumulate, and square root operations. It also provides conversions between fixed-point and floating-point data formats, and floating-point constant instructions.

Download Cortex-M4 Devices Generic User Guide, look for Floating Point Unit (FPU).

Move to topic: Enabling the FPU.

The FPU is disabled from reset. You must enable it before you can use any floating-point instructions.

## Compilation commands
### Compilation flags

    arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 main.c -o main.o
    arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 stm32f401_startup.c -o stm32f401_startup.o
    arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 syscalls.c -o syscalls.o

### Linker flags
When using standard libraries **-nostdlib** shall be removed from linker flags.

    arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb --specs=nano.specs -T stm32f401_ls.ld *.o -o stm32f401 -Wl,-Map=stm32f401.map

## Flashing.
### Command for open **openocd**

    openocd -f board/st_nucleo_f4.cfg

### Command to open telnet in a terminal

    telnet localhost 4444

### Preparing microcontroller for flashing.

    reset init

### Flashing microcontroller.

    flash write_image erase stm32f401

Again use:

    reset_init

Use resume to start running microcontroller:

    resume

Use exit to abort telnet connections.

    exit

## How to see USART frames in terminal.
### To see connected devices use: lsusb, it list all USB connected devices.

    lsusb

### To see tty connected devices.

    sudo dmesg | grep tty

### Display frames.

    screen /dev/ttyACM0 115200

In this case my devices is connected at ACM0 with a baudrate of 115200, according the USAR2 configuration in the application.