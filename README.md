SpiFlashMem
===========

Simple interface to SPI serial flash memory devices (e.g. Winbond
W25Q80BV) from Arduino.

This code is based on the Adafruit_TinyFlash library (available at
https://github.com/adafruit/Adafruit_TinyFlash) with significant
enough modifications that it felt reasonable to release it instead of
simply forking that code.  Many thanks to "ladyada" Limor Fried for
all her awesome work!

To use the library you can simply clone this repository into the
libraries subdirectory of your sketchbook.  See the Arduino guide for
installing new libraries for more details:
https://www.arduino.cc/en/Guide/Libraries

The library comes with a set of test cases under
examples/SpiFlashMemTestCases/SpiFlashMemTestCases.ino that were used
to validate the correct function of the code.  This will appear under
the examples in the Arduino sketchbook, but does not function as a
simple example for how to use the code, so here is a simple primer:

0. I prefer to avoid including header files within header files, so
   your sketch will need to include stdint.h before SpiFlash.h in
   order to compile correctly; this may change in the future.  In
   order to compile the library with debug printing on (see comments
   related to the DEBUG_WRITE preprocessor variable in SpiFlashMem.cpp
   for details) or to compile the test case program, you will need to
   obtain the StreamSerial library from the repository as well.

1. Construct an SpiFlashMem instance with the number of the Arduino
   pin that has been wired to /CS as its only argument.

2. Call the init() member function on the instance.  This should
   generally be done in the setup() function of the sketch.

3. Allocate a buffer for your data.

4. To read data from the device memory to the Arduino, call the read()
   member function with an address, a pointer to the buffer, and a
   count of bytes to read.

5. To write data from the Arduino to the device memory, call the
   write() member function, whose arguments are very similar to the
   read() function.

6. The minimum erase size is a sector (4096 bytes), and always occurs
   on a sector boundary.  The eraseSector() member function erases by
   sector number; to get the starting address of a sector, multiply
   the sector number by 4096 (or call the sector2BaseAddress() member
   function).

7. The eraseChip() member function will erase all data on the chip.

8. All member functions which can fail will return an 8 bit (uint8_t)
   code.  If the return code is equal to SpiFlashMem::SUCCESS_RESULT,
   then the function succeeded correctly; otherwise, the
   getErrorString() member function call be called with return code
   value and it will return a pointer to a more descriptive string
   stored in Arduino Flash memory.
