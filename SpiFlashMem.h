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
 * W25Q80BV:
 *
 * pin 1 (/CS) is active low chip select; this should be the pin
 *       number argument to the constructor, and it also should be
 *       tied to 3.3V via a pullup resistor to prevent spurious
 *       modification during startup.
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

#ifndef SPIFLASHMEM_H
#define SPIFLASHMEM_H 1


class SpiFlashMem
{
public:
  // TODO
  // typedef const __FlashStringHelper * ErrorStringType;

  enum {
    // externally visible constants
    CHIP_TOTAL_BYTES = 1L * 1024L * 1024L,
    CHIP_PAGE_SIZE = 256, // 2^8
    CHIP_SECTOR_SIZE = 4096, // 2^12
    // return codes
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

  typedef uint32_t Address;

  SpiFlashMem(const uint8_t chipSelectPin);

  uint8_t
  init();

  uint8_t
  read(const Address address,
       uint8_t *buffer,
       const uint32_t count) const;

  uint8_t
  write(const Address address,
	const uint8_t *buffer,
	const uint32_t count);

  uint8_t
  eraseSector(const uint16_t sector);

  uint8_t
  eraseChip();

  static inline Address
  sector2BaseAddress(const uint16_t sector)
  {
    return static_cast<Address>(sector) << SECTOR_2_ADDRESS_SHIFT;
  }

  static inline uint8_t
  address2Sector(const Address address)
  {
    return static_cast<uint8_t>(address >> SECTOR_2_ADDRESS_SHIFT);
  }

  static inline Address
  page2BaseAddress(const uint16_t page)
  {
    return static_cast<Address>(page) >> PAGE_2_ADDRESS_SHIFT;
  }

  static inline uint16_t
  address2Page(const Address address)
  {
    return static_cast<uint16_t>((address >> PAGE_2_ADDRESS_SHIFT) &
				 SIXTEEN_BIT_MASK);
  }

  static inline Address
  nextPageAddress(const Address address)
  {
    return (((address >> PAGE_2_ADDRESS_SHIFT) + 1) << PAGE_2_ADDRESS_SHIFT);
  }

  static inline Address
  previousPageAddress(const Address address)
  {
    return (((address >> PAGE_2_ADDRESS_SHIFT) - 1) << PAGE_2_ADDRESS_SHIFT);
  }

  // TODO
  // static inline ErrorStringType
  // getErrorString(const uint8_t returnCode)
  // {
  //   return (returnCode >= MAX_ERRORS) ? F("unknown") : RETURN_CODES[returnCode];
  // }

private:
  // TODO
  // static ErrorStringType RETURN_CODES[MAX_ERRORS];

  enum {
    // masks
    EIGHT_BIT_MASK = 255,
    SIXTEEN_BIT_MASK = (EIGHT_BIT_MASK << 8) & EIGHT_BIT_MASK,
    // various useful sizes
    CHIP_TOTAL_SECTORS = CHIP_TOTAL_BYTES / CHIP_SECTOR_SIZE,
    CHIP_TOTAL_PAGES = CHIP_TOTAL_BYTES / CHIP_PAGE_SIZE,
    CHIP_PAGES_PER_SECTOR = CHIP_SECTOR_SIZE / CHIP_PAGE_SIZE,
    SECTOR_2_ADDRESS_SHIFT = 12,
    PAGE_2_ADDRESS_SHIFT = 8,

    // chip-specific information from the datasheet

    // identifiers used for validation
    MANUFACTURER_ID = 0xEF,
    DEVICE_ID = 0x13,
    // commands
    PROGRAM_PAGE_CMD = 0x02,
    READ_DATA_CMD = 0x03,
    WRITE_DISABLE_CMD = 0x04,
    READ_STATUS_CMD = 0x05,
    WRITE_ENABLE_CMD = 0x06,
    ERASE_SECTOR_CMD = 0x20,
    ERASE_CHIP_CMD = 0x60,
    GET_ID_CMD = 0x90,
    // status
    BUSY_STATUS = 0x01,
    WRITE_ENABLED_STATUS = 0x02
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
