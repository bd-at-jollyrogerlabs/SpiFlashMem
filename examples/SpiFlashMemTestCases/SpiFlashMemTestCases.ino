/** -*- mode: c++ -*-
 *
 * SpiFlashMemTestCases.ino 
 *
 * Test cases for SpiFlashMem library.
 *
 * NOTE: these cases will produce some wear (but intended to only be a
 * small amount) on the physical chip under test.
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

#include <stdint.h>
#include <SPI.h>
#include <StreamSerial.h>
#include "SpiFlashMem.h"

typedef SpiFlashMem::Address Address;

// Standard pin assignments for Arduino SPI hardware
#ifdef __AVR_ATmega328P__
enum {
  MOSI_PIN = 11,
  MISO_PIN = 12,
  SCK_PIN  = 13,
};
#else
#ifdef __AVR_ATmega2560__
enum {
  MOSI_PIN = 51,
  MISO_PIN = 50,
  SCK_PIN  = 52,
};
#else
#error Unknown processor type, unable to determine SPI pin assignments
#endif
#endif

enum {
  CHIP_TOTAL_BYTES = SpiFlashMem::CHIP_TOTAL_BYTES,
  CHIP_PAGE_SIZE = SpiFlashMem::CHIP_PAGE_SIZE,
  CHIP_SECTOR_SIZE = SpiFlashMem::CHIP_SECTOR_SIZE,
  CHIP_TOTAL_SECTORS = CHIP_TOTAL_BYTES / CHIP_SECTOR_SIZE,
  CHIP_TOTAL_PAGES = CHIP_TOTAL_BYTES / CHIP_PAGE_SIZE,
  // NOTE: this should be defined in stdint.h
  UINT8_MAX = 255,
  RANDOM_SEED_PIN = A0,
  SERIAL_FLASH_CHIP_SELECT_PIN = 10
};

// TODO: find a way to embed this data within SpiFlashMem while
// keeping the strings in program memory.
static const char *ERROR_STRINGS[SpiFlashMem::MAX_ERRORS] = {
  "Success",
  "Timeout",
  "Bad Address",
  "Write Error",
  "Partial Write Error",
  "Identifier Mismatch",
  "Not Initialized",
  "Internal Error"
};

#define EXPECT_SUCCESS(...)					\
  do {								\
    uint8_t result = __VA_ARGS__;				\
    if (SpiFlashMem::SUCCESS_RESULT != result) {		\
      Serial << "ERROR: " << ERROR_STRINGS[result] << endl;	\
      return;							\
    }								\
    Serial << "OK" << endl;					\
  } while (0)

static uint8_t
findTestAddress(SpiFlashMem &mem,
		// TODO: figure out why scope resolution is required
		// here.
		SpiFlashMem::Address minAddress,
		Address maxAddress,
		Address *result)
{
  *result = minAddress;
  uint8_t returnCode = SpiFlashMem::SUCCESS_RESULT;
  for (Address address = minAddress; address <= maxAddress; address += 256) {
    uint8_t values[256];
    const uint32_t readCount = ((address + 256) <= maxAddress) ? 256 : (maxAddress - address);
    memset(values, 0, readCount);
    returnCode = mem.read(address, values, readCount);
    if (SpiFlashMem::SUCCESS_RESULT != returnCode) {
      goto EXIT;
    }
    for (size_t index = 0; index < readCount; ++index) {
      if (UINT8_MAX != values[index]) {
	*result = address + static_cast<Address>(index);
	goto EXIT;
      }
    }
  }
  Serial << "OK" << endl;
  // No byte with a cleared bit was found, write a 0 value into
  // the test address in order to be able to check if chip was
  // successfully erased.
  Serial << "Writing a test value into address "
	 << *result << "...";
  {
    const Address testAddress = *result;
    const uint8_t testValue = 0;
    returnCode = mem.write(testAddress, &testValue, 1);
  }
  if (SpiFlashMem::SUCCESS_RESULT != returnCode) {
    goto EXIT;
  }

 EXIT:
  return returnCode;
}

static uint8_t
validateErasedByte(const SpiFlashMem &mem,
		   // TODO: figure out why scope resolution is
		   // required here.
		   const SpiFlashMem::Address testAddress,
		   const char * const erasureType)
{
  Serial << "Reading back test value to validate " << erasureType << " erasure...";
  uint8_t testValue;
  uint8_t returnCode = mem.read(testAddress, &testValue, 1);
  if (SpiFlashMem::SUCCESS_RESULT != returnCode) {
    goto EXIT;
  }
  Serial << "OK" << endl << "Checking resulting value...";
  returnCode = SpiFlashMem::SUCCESS_RESULT;
  if (UINT8_MAX != testValue) {
    // TODO: should use a constant from SpiFlashMem here...
    returnCode = 7;  // i.e. "Internal Error"
  }

 EXIT:
  return returnCode;
}

void
setup()
{
  // Infrastructure initialization.
  Serial.begin(9600);
  SpiFlashMem mem(SERIAL_FLASH_CHIP_SELECT_PIN);
  // NOTE: assuming that analog pin 0 is not connected so that random
  // noise is read here
  randomSeed(analogRead(RANDOM_SEED_PIN));

  // Test cases

  // Test initialization
  Serial << "Initializing...";
  EXPECT_SUCCESS(mem.init());

  // Test chip erase
  {
    // First, search for the first byte which has a cleared bit.
    Serial << "Finding a test byte for use in chip erase test...";
    Address testAddress;
    EXPECT_SUCCESS(findTestAddress(mem, 0, CHIP_TOTAL_BYTES - 1, &testAddress));

    Serial << "Erasing entire chip...";
    EXPECT_SUCCESS(mem.eraseChip());

    EXPECT_SUCCESS(validateErasedByte(mem, testAddress, "chip"));
  }

  // Test Sector erase
  {
    const uint16_t sector = random(CHIP_TOTAL_SECTORS);
    Serial << "Finding a test byte for use in sector erase test...";
    Address testAddress;
    EXPECT_SUCCESS(findTestAddress(mem, sector * CHIP_SECTOR_SIZE,
  				   (sector * CHIP_SECTOR_SIZE) + CHIP_SECTOR_SIZE - 1,
  				   &testAddress));

    Serial << "Erasing sector " << sector << "...";
    EXPECT_SUCCESS(mem.eraseSector(sector));

    EXPECT_SUCCESS(validateErasedByte(mem, testAddress, "sector"));
  }

  // TODO
  // Test write within a single page

  // TODO
  // Test write which crosses page boundaries

  // TODO
  // Check that attempting to read beyond max address produces an
  // error

  // TODO
  // Check that attempting to write beyond max address produces an
  // error

  Serial << "Successful result for all tests" << endl;
}

void
loop()
{
  Serial << "Testing Completed" << endl;
  while (1) {
    delay(1000);
  }
}
