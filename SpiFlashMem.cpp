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

#include <stdint.h>
#include <SPI.h>
#include <StreamSerial.h>
#include "SpiFlashMem.h"

// TODO
// SpiFlashMem::ErrorStringType
// SpiFlashMem::RETURN_CODES[SpiFlashMem::MAX_ERRORS] = {
//   F("Success"),
//   F("Timeout"),
//   F("Bad Address"),
//   F("Write Error"),
//   F("Partial Write Error"),
//   F("Identifier Mismatch"),
//   F("Not Initialized"),
//   F("Internal Error")
// };

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

  uint8_t result = validateDeviceIdentifiers();
  if (SUCCESS_RESULT != result) {
    isInitialized_ = false;
    goto EXIT;
  }

  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
read(const Address address,
     uint8_t *buffer,
     const uint32_t count) const
{
  uint8_t result = ADDRESS_ERROR;
  if ((address + count) > CHIP_TOTAL_BYTES) {
    goto EXIT;
  }
  if (0 == count) {
    result = SUCCESS_RESULT;
    goto EXIT;
  }
  result = beginRead(address);
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  for (unsigned index = 0; index < count; ++index) {
    buffer[index] = SPI.transfer(0);
  }
  deselectChip();  // ends the read automatically
  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
write(const Address address,
      const uint8_t *buffer,
      const uint32_t count)
{
  uint32_t written = 0;
  Address startAddress = address;
  uint8_t result = ADDRESS_ERROR;
  if ((address + count) > CHIP_TOTAL_BYTES) {
    goto EXIT;
  }
  if (0 == count) {
    result = SUCCESS_RESULT;
    goto EXIT;
  }
  while (written < count) {
    // NOTE: data writes may not cross page boundaries.
    const Address nextPageBase = SpiFlashMem::nextPageAddress(startAddress);
    // TODO:
    // Serial << "Next page base is " << nextPageBase << endl;
    // Calculate the number of bytes to be written in this
    // iteration.
    uint32_t writeCount = count - written;
    if ((startAddress + writeCount) >= nextPageBase) {
      writeCount = (nextPageBase - 1) - startAddress;
    }
    // TODO:
    // Serial << "Writing " << writeCount << " bytes to this page" << endl;
    // Enable writing
    result = beginWrite();
    if (SUCCESS_RESULT != result) {
      if (written > 0) {
	result = PARTIAL_WRITE_ERROR;
      }
      goto EXIT;
    }
    // Send "Page Program" command + address
    // TODO:
    // Serial << "Sending page program command + address" << endl;
    sendCommand(PROGRAM_PAGE_CMD);
    SPI.transfer((startAddress >> 16) & EIGHT_BIT_MASK);
    SPI.transfer((startAddress >> 8) & EIGHT_BIT_MASK);
    SPI.transfer(startAddress & EIGHT_BIT_MASK);
    // TODO:
    // Serial << "Sending data" << endl;
    // Send data to the on-chip buffer
    for (uint32_t counter = 0; counter < writeCount; ++counter) {
      SPI.transfer(buffer[written + counter]);
    }
    // Deselect chip to trigger the "Page Program" instruction to
    // read the data from the on-chip buffer and write it to the
    // non-volatile memory.
    // TODO:
    // Serial << "De-selecting chip" << endl;
    deselectChip();
    delay(3);
    // Wait until the BUSY bit is cleared; "write enable latch"
    // (WEL) should also be cleared, but this is not currently
    // checked.
    // TODO:
    // Serial << "Waiting for ready" << endl;
    result = waitForReady();
    if (SUCCESS_RESULT != result) {
      if (written > 0) {
	result = PARTIAL_WRITE_ERROR;
      }
      goto EXIT;
    }
    startAddress += writeCount;
    written += writeCount;
    // TODO:
    // Serial << "Written = " << written << ", count = " << count << endl;
  }

  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
eraseSector(const uint16_t sector)
{
  uint8_t result = ADDRESS_ERROR;
  if (sector > (CHIP_TOTAL_SECTORS - 1)) {
    goto EXIT;
  }
  result = beginWrite();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  sendCommand(ERASE_SECTOR_CMD);
  SPI.transfer((sector >> 8) & EIGHT_BIT_MASK);
  SPI.transfer(sector & EIGHT_BIT_MASK);
  SPI.transfer(0);  // lowest order bits are ignored
  deselectChip();
  result = waitForReady();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  endWrite();
  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
eraseChip()
{
  uint8_t result = beginWrite();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  sendCommand(ERASE_CHIP_CMD);
  deselectChip();
  result = waitForReady();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  endWrite();
  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
waitForReady(const uint32_t timeout) const
{
  uint8_t status;
  uint32_t startTime = millis();

  do {
    sendCommand(READ_STATUS_CMD);
    status = SPI.transfer(0);
    deselectChip();
    if ((millis() - startTime) > timeout) {
      return TIMEOUT_ERROR;
    }
  } while (status & BUSY_STATUS);
  return SUCCESS_RESULT;
}

uint8_t
SpiFlashMem::
beginRead(const Address address) const
{
  uint8_t result = ADDRESS_ERROR;
  if (address > (CHIP_TOTAL_BYTES - 1)) {
    goto EXIT;
  }
  result = waitForReady();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  sendCommand(READ_DATA_CMD);
  SPI.transfer((address >> 16) & EIGHT_BIT_MASK);
  SPI.transfer((address >> 8) & EIGHT_BIT_MASK);
  SPI.transfer(address & EIGHT_BIT_MASK);
  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
beginWrite()
{
  uint8_t result = waitForReady();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  sendCommand(WRITE_ENABLE_CMD);
  deselectChip();
  sendCommand(READ_STATUS_CMD);
  result = SPI.transfer(0);
  if (WRITE_ENABLED_STATUS != result) {
    result = WRITE_ERROR;
    goto EXIT;
  }
  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

void
SpiFlashMem::
endWrite()
{
  sendCommand(WRITE_DISABLE_CMD);
  deselectChip();
}

uint8_t
SpiFlashMem::
validateDeviceIdentifiers() const
{
  sendCommand(GET_ID_CMD);
  uint8_t manufacturerId;
  // NOTE: 3 dummy bytes precede the manufacturer ID.
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
