/** -*- mode: c++ -*-
 *
 * SpiFlashMem.cpp 
 *
 * Implementation of simple interface to SPI serial flash memory
 * devices (e.g. Winbond W25Q80BV).
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

// NOTE: uncomment the following line to turn on DEBUG logging for the
// write() member function.
//#define DEBUG_WRITE 1

#include <stdint.h>
#include <SPI.h>
#include "SpiFlashMem.h"

#ifdef DEBUG_WRITE
#include <StreamSerial.h>
#endif // DEBUG_WRITE

const char SUCCESS_STRING[] PROGMEM = "success";
const char TIMEOUT_STRING[] PROGMEM = "timeout";
const char BAD_ADDRESS_STRING[] PROGMEM = "bad address";
const char WRITE_ERROR_STRING[] PROGMEM = "write error";
const char PARTIAL_WRITE_ERROR_STRING[] PROGMEM = "partial write error";
const char IDENTIFIER_MISMATCH_STRING[] PROGMEM = "identifier mismatch";
const char NOT_INITIALIZED_STRING[] PROGMEM = "not initialized";
const char INTERNAL_ERROR_STRING[] PROGMEM = "internal error";

const char * const
SpiFlashMem::
ERROR_STRINGS[SpiFlashMem::MAX_ERRORS] = {
  SUCCESS_STRING,
  TIMEOUT_STRING,
  BAD_ADDRESS_STRING,
  WRITE_ERROR_STRING,
  PARTIAL_WRITE_ERROR_STRING,
  IDENTIFIER_MISMATCH_STRING,
  NOT_INITIALIZED_STRING,
  INTERNAL_ERROR_STRING
};

/**
 * NOTE: REQUIRE_INIT assumes that the surrounding block has defined a
 * variable named returnCode and also has an available EXIT label to
 * jump to.
 */
#define REQUIRE_INIT()				\
  do {						\
    if (!isInitialized_) {			\
      returnCode = NOT_INITIALIZED_ERROR;	\
    }						\
  } while (0)

/**
 * NOTE: EXIT_ON_FAIL assumes that the surrounding block has defined a
 * variable named returnCode and also has an available EXIT label to
 * jump to.
 */
#define EXIT_ON_FAIL(...)			\
  do {						\
    returnCode = __VA_ARGS__;			\
    if (SUCCESS_RESULT != returnCode) {		\
      goto EXIT;				\
    }						\
  } while(0)

SpiFlashMem::
SpiFlashMem(const uint8_t chipSelectPin)
  : chipSelectMask_(digitalPinToBitMask(chipSelectPin)),
    chipSelectPort_(portOutputRegister(digitalPinToPort(chipSelectPin))),
    isInitialized_(false)
{
  // NOTE: easiest to put this initialization here since the value of
  // chipSelectPin is converted into a port/mask combination for
  // future use and the original value is lost.
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
}

uint8_t
SpiFlashMem::
init()
{
  // Set the flag at the beginning to allow internal calls to public
  // member functions to succeed; this value must be changed if there
  // is an error.
  isInitialized_ = true;

  // NOTE: SPI.begin() may be called multiple times
  SPI.begin();

  uint8_t returnCode = validateDeviceIdentifiers();
  if (SUCCESS_RESULT != returnCode) {
    isInitialized_ = false;
    goto EXIT;
  }

  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

uint8_t
SpiFlashMem::
read(const Address address,
     uint8_t *buffer,
     const uint32_t count) const
{
  // FIXME: sanity checks on buffer buffer pointer
  uint8_t returnCode = SUCCESS_RESULT;
  REQUIRE_INIT();
  if ((address + count) > CHIP_TOTAL_BYTES) {
    returnCode = ADDRESS_ERROR;
    goto EXIT;
  }
  if (0 == count) {
    returnCode = SUCCESS_RESULT;
    goto EXIT;
  }
  EXIT_ON_FAIL(beginRead(address));
  for (unsigned index = 0; index < count; ++index) {
    buffer[index] = SPI.transfer(0);
  }
  deselectChip();  // ends the read automatically
  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

uint8_t
SpiFlashMem::
write(const Address address,
      const uint8_t *buffer,
      const uint32_t count)
{
  // FIXME: sanity checks on buffer buffer pointer
  uint32_t written = 0;
  Address startAddress = address;
  uint8_t returnCode;
  REQUIRE_INIT();
  if ((address + count) > CHIP_TOTAL_BYTES) {
    returnCode = ADDRESS_ERROR;
    goto EXIT;
  }
  if (0 == count) {
    returnCode = SUCCESS_RESULT;
    goto EXIT;
  }

# ifdef DEBUG_WRITE
  Serial << endl << F("\tDBG: Write address is ") << address << F(", count is ")
	 << count << endl;
# endif

  while (written < count) {
    // NOTE: data writes may not cross page boundaries; also, address
    // + count has already been checked against CHIP_TOTAL_BYTES, so
    // no need to check if the total memory will be exceeded here.
    const Address nextPageBase = (startAddress >= LAST_PAGE_START_ADDRESS) ?
      CHIP_TOTAL_BYTES :
      nextPageBaseAddress(startAddress);

#   ifdef DEBUG_WRITE
    Serial << F("\tDBG: Next page base for start address ") << startAddress
	   << F(" is ") << nextPageBase << endl;
#   endif

    // Calculate the number of bytes to be written in this
    // iteration.
    uint32_t writeCount = count - written;
    if ((startAddress + writeCount) >= nextPageBase) {
      writeCount = nextPageBase - startAddress;
    }

#   ifdef DEBUG_WRITE
    Serial << F("\tDBG: Writing ") << writeCount << F(" bytes to this page")
	   << endl;
#   endif

    // Enable writing
    returnCode = beginWrite();
    if (SUCCESS_RESULT != returnCode) {
      returnCode = (written > 0) ? PARTIAL_WRITE_ERROR : WRITE_ERROR;
      goto EXIT;
    }
    // Send "Page Program" command + address
    sendCommand(PAGE_PROGRAM_CMD);
    SPI.transfer(static_cast<uint8_t>((startAddress >> 16) & EIGHT_BIT_MASK));
    SPI.transfer(static_cast<uint8_t>((startAddress >> 8) & EIGHT_BIT_MASK));
    SPI.transfer(static_cast<uint8_t>(startAddress & EIGHT_BIT_MASK));

    // Send data to the on-chip buffer
    for (uint32_t counter = 0; counter < writeCount; ++counter) {
      SPI.transfer(buffer[written + counter]);
    }
    // Deselect chip to trigger the "Page Program" instruction to
    // read the data from the on-chip buffer and write it to the
    // non-volatile memory.
    deselectChip();
    // Wait until the BUSY bit is cleared.
    returnCode = waitForReady();
    if (SUCCESS_RESULT != returnCode) {
      returnCode = (written > 0) ? PARTIAL_WRITE_ERROR : WRITE_ERROR;
      goto EXIT;
    }
    // Force the write to end.
    endWrite();
    if (isWriteEnableLatchSet()) {
      // NOTE: assuming that it is an error for WEL to be set after
      // successfully sending "Write Disable Command"
      returnCode = (written > 0) ? PARTIAL_WRITE_ERROR : WRITE_ERROR;
      goto EXIT;
    }
    startAddress += writeCount;
    written += writeCount;

#   ifdef DEBUG_WRITE
    Serial << F("\tDBG: Written = ") << written << F(", count = ") << count
	   << F(", new start address = ") << startAddress << endl;
#   endif
  }

  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

uint8_t
SpiFlashMem::
eraseSector(const uint16_t sector)
{
  const uint32_t baseAddress =
    static_cast<uint32_t>(sector) * CHIP_SECTOR_SIZE;
  uint8_t returnCode;
  REQUIRE_INIT();
  if (sector > (CHIP_TOTAL_SECTORS - 1)) {
    returnCode = ADDRESS_ERROR;
    goto EXIT;
  }
  EXIT_ON_FAIL(beginWrite());
  sendCommand(ERASE_SECTOR_CMD);
  SPI.transfer(static_cast<uint8_t>((baseAddress >> 16) & EIGHT_BIT_MASK));
  SPI.transfer(static_cast<uint8_t>((baseAddress >> 8) & EIGHT_BIT_MASK));
  SPI.transfer(0);  // lowest order bits are ignored
  deselectChip();
  // NOTE: using timeout value from Adafruit_TinyFlash library.
  // Datasheet says 400 ms max
  EXIT_ON_FAIL(waitForReady(10000L));
  endWrite();
  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

uint8_t
SpiFlashMem::
eraseChip()
{
  uint8_t returnCode;
  REQUIRE_INIT();
  EXIT_ON_FAIL(beginWrite());
  sendCommand(ERASE_CHIP_CMD);
  deselectChip();
  // NOTE: using timeout value from Adafruit_TinyFlash library.
  // Datasheet says 6 sec max
  EXIT_ON_FAIL(waitForReady(10000L));
  endWrite();
  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

SpiFlashMem::ErrorStringType
SpiFlashMem::
getErrorString(const uint8_t returnCode)
{
  return (returnCode >= MAX_ERRORS) ? F("unknown") :
    reinterpret_cast<ErrorStringType>(ERROR_STRINGS[returnCode]);
}

uint8_t
SpiFlashMem::
waitForReady(const uint32_t timeout) const
{
  uint8_t status;
  uint32_t startTime = millis();

  do {
    sendCommand(READ_STATUS_REGISTER_1_CMD);
    status = SPI.transfer(0);
    deselectChip();
    if ((millis() - startTime) > timeout) {
      return TIMEOUT_ERROR;
    }
  } while (status & BUSY_STATUS);
  return SUCCESS_RESULT;
}

bool
SpiFlashMem::
isWriteEnableLatchSet() const
{
  uint8_t status;
  sendCommand(READ_STATUS_REGISTER_1_CMD);
  status = SPI.transfer(0);
  deselectChip();
  return status & WRITE_ENABLED_STATUS;
}

uint8_t
SpiFlashMem::
beginRead(const Address address) const
{
  uint8_t returnCode = ADDRESS_ERROR;
  if (address > (CHIP_TOTAL_BYTES - 1)) {
    goto EXIT;
  }
  EXIT_ON_FAIL(waitForReady());
  sendCommand(READ_DATA_CMD);
  SPI.transfer(static_cast<uint8_t>((address >> 16) & EIGHT_BIT_MASK));
  SPI.transfer(static_cast<uint8_t>((address >> 8) & EIGHT_BIT_MASK));
  SPI.transfer(static_cast<uint8_t>(address & EIGHT_BIT_MASK));
  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

uint8_t
SpiFlashMem::
beginWrite()
{
  uint8_t returnCode;
  EXIT_ON_FAIL(waitForReady());
  // Send "Write Enable" command.
  sendCommand(WRITE_ENABLE_CMD);
  deselectChip();
  // Send "Read Status Register-1" command.
  sendCommand(READ_STATUS_REGISTER_1_CMD);
  returnCode = SPI.transfer(0);
  deselectChip();
  // Check if "Write Enable Latch" (WEL) is set.
  if (!(WRITE_ENABLED_STATUS & returnCode)) {
    returnCode = WRITE_ERROR;
    goto EXIT;
  }
  returnCode = SUCCESS_RESULT;

 EXIT:
  return returnCode;
}

void
SpiFlashMem::
endWrite()
{
  // Send "Write Disable" command.
  sendCommand(WRITE_DISABLE_CMD);
  deselectChip();
  // MAYBE: maybe check the value of WEL?
}

uint8_t
SpiFlashMem::
validateDeviceIdentifiers() const
{
  sendCommand(READ_MANUFACTURER_AND_DEVICE_ID_CMD);
  uint8_t manufacturerId;
  // NOTE: 3 bytes set to 0 must be sent as the address part of the
  // command before the manufacturer and device IDs can be read.
  for (uint8_t counter = 0; counter < 4; ++counter) {
    manufacturerId = SPI.transfer(0);
  }
  uint8_t deviceId = SPI.transfer(0);
  deselectChip();

  if ((MANUFACTURER_ID != manufacturerId) ||
      (DEVICE_ID != deviceId)) {
    return IDENTIFIER_MISMATCH_ERROR;
  }
  return SUCCESS_RESULT;
}
