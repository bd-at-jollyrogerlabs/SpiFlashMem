/** -*- mode: c++ -*-
 *
 * SpiFlashMem.h 
 *
 * Simple interface to SPI serial flash memory devices (e.g. Winbond
 * W25Q80BV).
 *
 * NOTE: this code is partially based on the Adafruit_TinyFlash
 * library available at
 * https://github.com/adafruit/Adafruit_TinyFlash.
 * 
 * Copyright (c) 2015, Brian Davis
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Brian Davis <bd@jollyrogerlabs.com>
 * 
 */

/*
 *
 * Pin assignment documentation, based based on the datasheet for
 * the Winbond W25Q80BV (revision 1, released 9-oct-2013):
 *
 * pin 1 (/CS) is active low chip select; this should be the pin
 *       number argument to the constructor, and it also should be
 *       tied to Vcc via a pullup resistor to prevent spurious
 *       modification during startup, as per the datasheet.
 *
 * pin 2 (DO AKA IO1) is MISO (pin 12 on Arduino Uno or pin 50 on
 *       Arduino Mega2560).
 *
 * pin 3 (/WP AKA IO2) is used for active low write protection or Quad
 *       SPI instructions and should thus be tied to 3.3V if this
 *       functionality is not used (this library does not support it).
 *
 * pin 5 (DI AKA IO0) is MOSI (pin 11 on Arduino Uno or pin 51 on
 *       Arduino Mega2560).
 *
 * pin 6 (CLK) is SCK (pin 13 on Arduino Uno or pin 52 on Arduino
 *       Mega2560).
 *
 * pin 7 (/HOLD AKA IO3) is used for active low hold or Quad SPI
 *       instructions and should be tied to 3.3V is this functionality
 *       is not used (this library does not support it).
 */

/*
 * Current pending FIXME fixes:
 *
 * 1. Improve error checking on function arguments, especially
 *    checking for NULL on pointer arguments.  This will require a new
 *    return code constant.
 *
 * 2. Create separate types for addresses, sector IDs, and page IDs,
 *    using "unit-safe" methodology.
 */

/*
 * Possible future features, in rough order of importance:
 *
 * - Conditionally place some error strings in the flash memory itself
 *   to handle the case where program memory is scarce.
 * 
 * - Conditional definition of SpiFlashMem type from various types of
 *   chips other than W25Q80BV.
 *
 * - Support for "chip select policy" to allow a chip select mechanism
 *   other than having a single pin assigned to control a single chip
 *   (e.g. shift register); "chip select traits" would be a template
 *   argument to the SpiFlashMem class.
 *
 * - Support for active low hold pin to allow transmission of large
 *    blocks of data to be paused in order to service other components
 *    attached to the SPI bus.
 *
 * - Support for "Power-down" instruction to allow improved battery
 *   life.
 *
 * - Support for asynchronous completion of longer operations such as
 *   chip or sector erase; this would require support from some
 *   external mechanism, most likely via some template hackery.
 *
 * - Support for various "write protect" functions that are provided
 *   by the chip and accessed by sending instructions over SPI.
 *
 * - Support for returning manufacturer and device ID to client code
 *   as well as returning the device Unique ID Number and JEDEC ID;
 *   manufacturer and device ID are currently validated internally via
 *   compile time constants, but validation could also occur by
 *   supplying some sort of "ID traits" type via template argument.
 *
 * - Support for various block erase instructions for sizes larger
 *   than a sector (32k, 64k); should be easy to add.
 *
 * - Support for "Erase/Program Suspend" and "Erase/Program Resume" to
 *   allow data to flow to or from sectors that are not currently
 *   being erased.
 *
 * - Support for "Serial Flash Discoverable Parameter" (SFDP) data
 *   which could be used to make the entire class work with any
 *   supported chip in a Plug'n'Play fashion.
 *
 * Features that will probably not be supported:
 *
 * - Dual and Quad SPI instructions, since this is not supported in
 *   hardware on Atmel microcontrollers used for Arduino.
 */

#ifndef SPIFLASHMEM_H
#define SPIFLASHMEM_H 1

class SpiFlashMem
{
public:
  // Main enum containing integer constants.
  enum {
    // externally visible constants
    CHIP_TOTAL_BYTES = 1L * 1024L * 1024L,
    CHIP_PAGE_SIZE = 256, // 2^8, from datasheet
    CHIP_SECTOR_SIZE = 4096, // 2^12
    CHIP_TOTAL_SECTORS = CHIP_TOTAL_BYTES / CHIP_SECTOR_SIZE,
    CHIP_MAX_ERASE_COUNT = 100000,

    // standard return codes
    SUCCESS_RESULT = 0,
    TIMEOUT_ERROR = 1,
    ADDRESS_ERROR = 2,
    WRITE_ERROR = 3,
    PARTIAL_WRITE_ERROR = 4,
    IDENTIFIER_MISMATCH_ERROR = 5,
    NOT_INITIALIZED_ERROR = 6,
    INTERNAL_ERROR = 7,
    MAX_ERRORS = 8
  };

  typedef const __FlashStringHelper * ErrorStringType;
  typedef uint32_t Address;
  typedef uint16_t Sector;
  typedef uint16_t Page;

  SpiFlashMem(const uint8_t chipSelectPin);

  /**
   * Initialize the object for use, including the initialization of
   * the global SPI object.
   *
   * Returns a standard return code.
   */
  uint8_t
  init();

  /**
   * Return 'true' if the chip has been initialized, 'false'
   * otherwise.
   */
  inline bool
  isInitialized() const
  {
    return isInitialized_;
  }

  /**
   * Read a block of data starting at the argument address into a
   * provided buffer.
   *
   * Returns a standard return code.
   */
  uint8_t
  read(const Address address,
       uint8_t *buffer,
       const uint32_t count) const;

  /**
   * Write a block of data from a provided buffer into the memory
   * starting at the argument address.
   *
   * NOTE: this function will handle the crossing of page boundaries
   * correctly, but for best performance, large data writes should be
   * broken up into page-sized chunks and written to page-aligned
   * boundaries.
   *
   * Returns a standard return code.
   */
  uint8_t
  write(const Address address,
	const uint8_t *buffer,
	const uint32_t count);

  /**
   * Erase (i.e. set all bits in) the argument sector.
   *
   * Returns a standard return code.
   */
  uint8_t
  eraseSector(const Sector sector);

  /**
   * Erase (i.e. set all bits to 1 in) all sectors.
   *
   * Returns a standard return code.
   */
  uint8_t
  eraseChip();

  /**** Various useful address calculation functions. ****/

  /**
   * Note on argument range and return values:
   *
   * 1. WARNING: if argument sector is larger than the total number of
   *    sectors, the base address of the last sector is returned.
   */
  static inline Address
  sector2BaseAddress(const Sector sector)
  {
    return (sector > (CHIP_TOTAL_SECTORS - 1)) ?
      static_cast<Address>(CHIP_TOTAL_SECTORS - 1) << SECTOR_2_ADDRESS_SHIFT :
      static_cast<Address>(sector) << SECTOR_2_ADDRESS_SHIFT;
  }

  /**
   * Note on argument range and return values:
   *
   * 1. WARNING: if argument address is larger than the total number
   *    of bytes, the number of the last sector is returned.
   */
  static inline Sector
  address2Sector(const Address address)
  {
    return (address > (CHIP_TOTAL_BYTES - 1)) ?
      (CHIP_TOTAL_SECTORS - 1) :
      static_cast<uint8_t>(address >> SECTOR_2_ADDRESS_SHIFT);
  }

  /**
   * Note on argument range and return values:
   *
   * 1. WARNING: if argument page is larger than the total number of
   *    pages, the base address of the last page is returned.
   */
  static inline Address
  page2BaseAddress(const Page page)
  {
    return (page > (CHIP_TOTAL_PAGES - 1)) ?
      static_cast<Address>(CHIP_TOTAL_PAGES - 1) << PAGE_2_ADDRESS_SHIFT :
      static_cast<Address>(page) << PAGE_2_ADDRESS_SHIFT;
  }

  /**
   * Note on argument range and return values:
   *
   * 1. WARNING: if argument address is larger than the base address
   *    of the last page, the last page number is returned.
   */
  static inline Page
  address2Page(const Address address)
  {
    return (address > (CHIP_TOTAL_BYTES - 1)) ?
      (CHIP_TOTAL_PAGES - 1) :
      static_cast<Page>(address >> PAGE_2_ADDRESS_SHIFT);
  }

  /**
   * Note on argument range and return values:
   *
   * 1. WARNING: if argument address is larger than the base address
   *    of the last page, the base address of the last page is
   *    returned.
   *
   * FIXME: this is not a very good abstraction, and should be removed
   * from the interface.
   */
  static inline Address
  nextPageBaseAddress(const Address address)
  {
    return (address > (CHIP_TOTAL_BYTES - 1)) ?
      page2BaseAddress(CHIP_TOTAL_PAGES - 1) :
      page2BaseAddress(address2Page(address) + 1);
  }

  /**
   * Note on argument range and return values:
   *
   * 1. WARNING: if argument address is larger than the maximum number
   *    of bytes in the memory, the base address of the last page will
   *    be returned.
   *
   * MAYBE: maybe rewrite the interface to allow the address value to
   * be returned by reference and have the function generate a return
   * code as well.
   *
   * FIXME: this is not a very good abstraction, and should be removed
   * from the interface.
   */
  static inline Address
  previousPageBaseAddress(const Address address)
  {
    // NOTE: first ternary term explicitly avoids overflow from
    // subtracting 1 from unsigned 0.
    return (address < CHIP_PAGE_SIZE) ? 0 :
      // NOTE: second ternary term handles the case where the argument
      // address exceeds the number of available bytes on the chip.
      (address > (CHIP_TOTAL_BYTES - 1)) ?
      page2BaseAddress(CHIP_TOTAL_PAGES - 1) :
      // NOTE: 'else' condition of second ternary term directly
      // calculates the value; could also call
      // page2BaseAddress(address2Page(address) - 1), but this
      // generates extraneous checks which should be avoided (although
      // the optimizer would be expected to eliminate them..)
      (((address >> PAGE_2_ADDRESS_SHIFT) - 1) << PAGE_2_ADDRESS_SHIFT);
  }

  static ErrorStringType
  getErrorString(const uint8_t returnCode);

private:
  static const char * const ERROR_STRINGS[MAX_ERRORS];

  enum {
    // masks
    EIGHT_BIT_MASK = 255,
    SIXTEEN_BIT_MASK = (EIGHT_BIT_MASK << 8) & EIGHT_BIT_MASK,

    // various useful sizes
    CHIP_TOTAL_PAGES = CHIP_TOTAL_BYTES / CHIP_PAGE_SIZE,  // 4096, from datasheet
    CHIP_PAGES_PER_SECTOR = CHIP_SECTOR_SIZE / CHIP_PAGE_SIZE,  // 16, from datasheet
    SECTOR_2_ADDRESS_SHIFT = 12,
    PAGE_2_ADDRESS_SHIFT = 8,

    // other useful stuff
    LAST_PAGE_START_ADDRESS = CHIP_TOTAL_BYTES - CHIP_PAGE_SIZE,

    // chip-specific information from the datasheet

    // identifiers used for validation
    MANUFACTURER_ID = 0xEF,
    DEVICE_ID = 0x13,
    // commands
    PAGE_PROGRAM_CMD = 0x02,
    READ_DATA_CMD = 0x03,
    WRITE_DISABLE_CMD = 0x04,
    READ_STATUS_REGISTER_1_CMD = 0x05,
    WRITE_ENABLE_CMD = 0x06,
    ERASE_SECTOR_CMD = 0x20,
    READ_STATUS_REGISTER_2_CMD = 0x35,
    ERASE_CHIP_CMD = 0x60,
    ERASE_PROGRAM_SUSPEND_CMD = 0x75,
    ERASE_PROGRAM_RESUME_CMD = 0x7A,
    POWER_DOWN_CMD = 0xB9,
    RELEASE_POWER_DOWN_DEVICE_ID_CMD = 0xAB,
    READ_MANUFACTURER_AND_DEVICE_ID_CMD = 0x90,
    READ_UNIQUE_ID_NUMBER_CMD = 0x4B,
    READ_JEDEC_ID_CMD = 0x9F,
    READ_SFDP_REGISTER = 0x5A,
    // status (status register 1)
    BUSY_STATUS = 0x01,
    WRITE_ENABLED_STATUS = 0x02,
    // status (status register 2)
    SUSPEND_STATUS = 0x80
  };

  inline void
  selectChip() const
  {
    *chipSelectPort_ &= ~chipSelectMask_;
  }

  inline void
  deselectChip() const
  {
    *chipSelectPort_ |= chipSelectMask_;
  }

  inline void
  sendCommand(const uint8_t command) const
  {
    selectChip();
    // NOTE: return value ignored.
    SPI.transfer(command);
  }

  uint8_t
  waitForReady(const uint32_t timeout = 100L) const;

  bool
  isWriteEnableLatchSet() const;

  uint8_t
  beginRead(const Address address) const;

  uint8_t
  beginWrite();

  void
  endWrite();

  uint8_t
  validateDeviceIdentifiers() const;

  const uint8_t chipSelectMask_;
  volatile uint8_t *chipSelectPort_;
  bool isInitialized_;
};

#endif // SPIFLASHMEM_H
