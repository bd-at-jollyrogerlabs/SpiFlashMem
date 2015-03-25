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
  SINGLE_PAGE_WRITE_TEST_SIZE = 9, // chosen in a fairly arbitrary fashion
  TWO_PAGE_WRITE_TEST_SIZE = 265, // chosen in a fairly arbitrary fashion
  // NOTE: this should be defined in stdint.h
  UINT8_MAX = 255,
  RANDOM_SEED_PIN = A0,
  SERIAL_FLASH_CHIP_SELECT_PIN = 10
};

// FIXME: find a way to embed this data within SpiFlashMem while
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
      Serial << F("ERROR: ") << ERROR_STRINGS[result] << endl;	\
      return;							\
    }								\
    Serial << F("OK") << endl;					\
  } while (0)

#define EXPECT_FAILURE(testName, ...)				\
  do {								\
    uint8_t result = __VA_ARGS__;				\
    if (SpiFlashMem::SUCCESS_RESULT == result) {		\
      Serial << F("ERROR: ") << testName			\
	     << F(" succeeded unexpectedly") << endl;		\
      return;							\
    }								\
    Serial << F("OK") << endl;					\
  } while (0)

/**
 * NOTE: EXIT_ON_FAIL assumes that the surrounding block has defined a
 * variable named returnCode and also has an available EXIT label to
 * jump to.
 */
#define EXIT_ON_FAIL(...)				\
  do {							\
    returnCode = __VA_ARGS__;				\
    if (SpiFlashMem::SUCCESS_RESULT != returnCode) {	\
      goto EXIT;					\
    }							\
    Serial << F("OK") << endl;				\
  } while(0)

/**
 * Prepare for testing an erase operation by choosing a byte in the
 * test range (i.e. memory area to be erased) which has at least 1 bit
 * cleared.  If no such byte is available, write a value of 0 into the
 * first address in the test range.
 */
static uint8_t
prepareTestByte(SpiFlashMem &mem,
		// FIXME: figure out why scope resolution is required
		// here.
		SpiFlashMem::Address minAddress,
		Address maxAddress,
		Address *result)
{
  *result = minAddress;
  uint8_t returnCode = SpiFlashMem::SUCCESS_RESULT;
  // Read memory values in blocks of 256 to speed things up.
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
	// Encountered the first byte with at least 1 cleared bit.
	*result = address + static_cast<Address>(index);
	goto EXIT;
      }
    }
  }
  // NOTE: logging OK here because more information will be printed
  // about this test.
  Serial << F("OK") << endl;
  // No byte with a cleared bit was found, write a 0 value into the
  // first address in order to be able to check if chip was
  // successfully erased.
  Serial << F("Writing test value 0 into byte at address ")
	 << *result << F("...");
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

/**
 * Validate that an address contains a byte with all bits set; this
 * value should be known to previously contain some cleared bits
 * before an erase operation.
 */
static uint8_t
validateErasedByte(const SpiFlashMem &mem,
		   // FIXME: figure out why scope resolution is
		   // required here.
		   const SpiFlashMem::Address testAddress,
		   const char * const erasureType)
{
  Serial << F("Reading back test value from address ") << testAddress
	 << F(" to validate ") << erasureType << F(" erasure...");
  uint8_t testValue;
  uint8_t returnCode = SpiFlashMem::SUCCESS_RESULT;
  EXIT_ON_FAIL(mem.read(testAddress, &testValue, 1));
  Serial << F("Checking resulting value...");
  returnCode = SpiFlashMem::SUCCESS_RESULT;
  if (UINT8_MAX != testValue) {
    Serial << F("Mismatch, expected ") << static_cast<unsigned>(UINT8_MAX)
	   << F(", received ") << static_cast<unsigned>(testValue) << F(": ");
    // FIXME: should use a constant from SpiFlashMem here...
    returnCode = 7;  // i.e. "Internal Error"
  }

 EXIT:
  return returnCode;
}

/**
 * Write a block of data which does not match the old data to the
 * flash memory, then read back the result and validate that it is
 * correct.
 */
static uint8_t
testBlockWrite(SpiFlashMem &mem,
	       const SpiFlashMem::Address baseAddress,
	       const char * const testType)
{
  Serial << F("Reading ") << static_cast<unsigned>(SINGLE_PAGE_WRITE_TEST_SIZE)
	 << F(" bytes of original data starting at address ")
	 << baseAddress << F(" (page ") << SpiFlashMem::address2Page(baseAddress)
	 << F(") for ") << testType << F(" test...");
  uint8_t oldData[SINGLE_PAGE_WRITE_TEST_SIZE];
  memset(oldData, 0, SINGLE_PAGE_WRITE_TEST_SIZE);
  uint8_t returnCode = 0;
  EXIT_ON_FAIL(mem.read(baseAddress, oldData, SINGLE_PAGE_WRITE_TEST_SIZE));
  // Generate new values which to not match the old values.
  uint8_t newData[SINGLE_PAGE_WRITE_TEST_SIZE];
  for (uint8_t index = 0; index < SINGLE_PAGE_WRITE_TEST_SIZE; ++index) {
    newData[index] = (UINT8_MAX == oldData[index]) ? index : oldData[index] + 1;
  }
  // Write the new values back to the location.
  Serial << F("Writing new data for ") << testType << F(" test...");
  EXIT_ON_FAIL(mem.write(baseAddress, newData, SINGLE_PAGE_WRITE_TEST_SIZE));
  // Read the new data back.
  uint8_t checkData[SINGLE_PAGE_WRITE_TEST_SIZE];
  memset(checkData, 0, SINGLE_PAGE_WRITE_TEST_SIZE);
  Serial << F("Reading check data for ") << testType << F(" test...");
  EXIT_ON_FAIL(mem.read(baseAddress, checkData, SINGLE_PAGE_WRITE_TEST_SIZE));
  // Compare check data to expected values.
  Serial << F("Comparing check data to expected values...");
  for (uint8_t offset = 0; offset < SINGLE_PAGE_WRITE_TEST_SIZE; ++offset) {
    if (checkData[offset] != newData[offset]) {
      Serial << F("Mismatch at offset ") << offset << F(" from base address ")
	     << baseAddress << F(": expected ") << newData[offset]
	     << F(", received ") << checkData[offset] << F(": ");
      // FIXME: should use a constant from SpiFlashMem here...
      returnCode = 7;  // i.e. "Internal Error"
      goto EXIT;
    }
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
  Serial << F("Initializing...");
  EXPECT_SUCCESS(mem.init());

  // Test basic address calculations
  {
    Serial << F("*** Testing basic address calculation support code ")
	   << F("for pages ***") << endl;
    uint16_t page = 0;
    Address baseAddress = SpiFlashMem::page2BaseAddress(page);
    for (uint16_t counter = 1; counter < (CHIP_TOTAL_PAGES - 1); ++counter) {
      // Check that all addresses within the page map back to the
      // correct page number.
      for (Address offset = 1; offset < CHIP_PAGE_SIZE; ++offset) {
      	const uint16_t checkPage =
      	  SpiFlashMem::address2Page(baseAddress + offset);
      	if (checkPage != page) {
      	  Serial << F("Page address calculation error: ")
      		 << F("page number was incorrectly calculated from offset ")
      		 << offset << F(" as ") << checkPage << F(" vs correct value ")
      		 << page << endl;
      	  return;
      	}
      }
      // Check that SpiFlashMem::nextPageBaseAddress works correctly
      const uint16_t oldPage = page;
      const Address oldAddress = baseAddress;
      baseAddress = SpiFlashMem::nextPageBaseAddress(baseAddress);
      page = SpiFlashMem::address2Page(baseAddress);
      if (page != (oldPage + 1)) {
	Serial << F("Page address calculation error: ")
	       << F("from base address ") << oldAddress << F(" of page ")
	       << oldPage << F(", next page was incorrectly calculated as ")
	       << page << F(" with base address ") << baseAddress << endl;
	return;
      }
      // Check that SpiFlashMem::prevPageBaseAddress works correctly
      const Address prevPageBase = SpiFlashMem::previousPageBaseAddress(baseAddress);
      const uint16_t prevPage = SpiFlashMem::address2Page(prevPageBase);
      if (prevPage != oldPage) {
	Serial << F("Page address calculation error: ")
	       << F("previous page was incorrectly backed out from page ")
	       << page << F(" as ") << oldPage << F(" instead of ")
	       << prevPage << endl;
	return;
      }
    }
  }
  {
    Serial << F("*** Testing basic address calculation support code ")
	   << F("for sectors ***") << endl;
    uint16_t sector = 0;
    Address baseAddress = SpiFlashMem::sector2BaseAddress(sector);
    for (uint16_t counter = 1; counter < (CHIP_TOTAL_SECTORS - 1); ++counter) {
      for (Address offset = 1; offset < CHIP_SECTOR_SIZE; ++offset) {
	const uint16_t checkSector =
	  SpiFlashMem::address2Sector(baseAddress + offset);
	if (checkSector != sector) {
	  Serial << F("Sector address calculation error: ")
		 << F("sector number was incorrectly calculated from offset ")
		 << offset << F(" as ") << checkSector << F(" vs correct value ")
		 << sector << endl;
	}
      }
      ++sector;
      baseAddress = SpiFlashMem::sector2BaseAddress(sector);
    }
  }

  // Test single byte write
  {
    Serial << F("*** Testing single byte write ***") << endl;
    const Address testAddress = random(CHIP_TOTAL_BYTES - 1);
    Serial << F("Reading original byte value from address ") << testAddress
	   << F(" for single byte write test...");
    uint8_t oldValue = 0;
    EXPECT_SUCCESS(mem.read(testAddress, &oldValue, 1));
    Serial << F("Writing new byte value to address ") << testAddress
	   << F(" for single byte write test...");
    const uint8_t newValue = (UINT8_MAX == oldValue) ? 0 : oldValue + 1;
    EXPECT_SUCCESS(mem.write(testAddress, &newValue, 1));
    Serial << F("Reading check byte value from address ") << testAddress
	   << F(" for single byte write test...");
    uint8_t checkValue = oldValue;  // the one value that should not result
    EXPECT_SUCCESS(mem.read(testAddress, &checkValue, 1));
    Serial << F("Comparing check value ") << static_cast<unsigned>(checkValue)
	   << F(" from address ") << testAddress << F(" to written value ")
	   << static_cast<unsigned>(newValue) << F(" for single byte write test...");
    if (newValue != checkValue) {
      Serial << F("Mismatch") << endl;
      return;
    }
    Serial << F("OK") << endl;
  }

  // Test chip erase
  {
    Serial << F("*** Testing chip erase ***") << endl;
    // First, search for the first byte which has a cleared bit.
    Serial << F("Finding a test byte for use in chip erase test...");
    Address testAddress;
    EXPECT_SUCCESS(prepareTestByte(mem, 0, CHIP_TOTAL_BYTES - 1, &testAddress));

    Serial << F("Erasing entire chip...");
    EXPECT_SUCCESS(mem.eraseChip());

    EXPECT_SUCCESS(validateErasedByte(mem, testAddress, "chip"));
  }

  // Test Sector erase
  {
    Serial << F("*** Testing sector erase ***") << endl;
    const uint16_t sector = random(CHIP_TOTAL_SECTORS);
    const Address startAddress = static_cast<uint32_t>(sector) * CHIP_SECTOR_SIZE;
    const Address endAddress = startAddress + CHIP_SECTOR_SIZE - 1;
    Serial << F("Finding a test byte in sector ") << sector << F(" starting at address ")
	   << startAddress << F(" and ending at address ") << endAddress
	   << F(" for use in sector erase test...");
    Address testAddress;
    EXPECT_SUCCESS(prepareTestByte(mem, startAddress, endAddress,
  				   &testAddress));

    Serial << F("Erasing sector ") << sector << F("...");
    EXPECT_SUCCESS(mem.eraseSector(sector));

    EXPECT_SUCCESS(validateErasedByte(mem, testAddress, "sector"));
  }

  // Test write within a single page
  {
    Serial << F("*** Testing intra-page write ***") << endl;
    // Choose a random page and a random offset within the page
    // subject to the constraint that the test bytes written WILL NOT
    // cross the page boundary.
    const uint16_t page = random(CHIP_TOTAL_PAGES);
    const uint32_t offset = random(CHIP_PAGE_SIZE - (SINGLE_PAGE_WRITE_TEST_SIZE + 1));
    Address baseAddress =
      static_cast<Address>((static_cast<uint32_t>(page * CHIP_PAGE_SIZE) +
			    offset));
    EXPECT_SUCCESS(testBlockWrite(mem, baseAddress, "intra-page write"));
  }

  // Test write which crosses a single page boundary
  {
    Serial << F("*** Testing inter-page write ***") << endl;
    // Choose a random page and a random offset within the page
    // subject to the constraint that the test bytes written WILL
    // cross the page boundary.
    const uint16_t page = random(CHIP_TOTAL_PAGES - 1);
    const uint32_t offset = random(CHIP_PAGE_SIZE - (SINGLE_PAGE_WRITE_TEST_SIZE / 2),
				   CHIP_PAGE_SIZE - (SINGLE_PAGE_WRITE_TEST_SIZE / 3));
    
    Address baseAddress =
      SpiFlashMem::page2BaseAddress(page) + static_cast<Address>(offset);
    EXPECT_SUCCESS(testBlockWrite(mem, baseAddress, "inter-page write"));
  }

  // NOTE: not currently writing a test which crosses more than one
  // page boundary in a single call of write() due to the fact that
  // this would require the allocation of an array which is greater
  // than the page size, an occurrence which is not expected to happen
  // in practice due to the memory limitations on the targeted class
  // of microcontrollers.

  // Error handling tests.
  {
    Serial << F("*** Error handling tests ***") << endl;
    uint8_t dummy;
    // Check that attempting to read beyond max address produces an
    // error
    Serial << F("Attempting to read beyond max address should fail...");
    EXPECT_FAILURE("reading beyond max address",
		   mem.read(static_cast<Address>(CHIP_TOTAL_BYTES - 10),
			    &dummy, 20));
    // Check that attempting to write beyond max address produces an
    // error
    Serial << F("Attempting to write beyond max address should fail...");
    EXPECT_FAILURE("reading beyond max address",
		   mem.write(static_cast<Address>(CHIP_TOTAL_BYTES - 10),
			     &dummy, 20));
  }

  Serial << F("=== Successful result for all tests ===") << endl;
}

void
loop()
{
  Serial << F("Testing Completed") << endl;
  while (1) {
    delay(1000);
  }
}
