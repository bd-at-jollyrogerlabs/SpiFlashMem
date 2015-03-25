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

// NOTE: uncomment the following line to turn on DEBUG logging for the
// write() member function.
// #define DEBUG_WRITE 1

// FIXME - handle conversion from return codes to error strings
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

# ifdef DEBUG_WRITE
  Serial << endl << F("\tDBG: Write address is ") << address << F(", count is ")
	 << count << endl;
# endif

  while (written < count) {
    // NOTE: data writes may not cross page boundaries.
    const Address nextPageBase = SpiFlashMem::nextPageBaseAddress(startAddress);

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
    result = beginWrite();
    if (SUCCESS_RESULT != result) {
      result = (written > 0) ? PARTIAL_WRITE_ERROR : WRITE_ERROR;
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
    result = waitForReady();
    if (SUCCESS_RESULT != result) {
      result = (written > 0) ? PARTIAL_WRITE_ERROR : WRITE_ERROR;
      goto EXIT;
    }
    // Force the write to end.
    endWrite();
    if (isWriteEnableLatchSet()) {
      // NOTE: assuming that it is an error for WEL to be set after
      // successfully sending "Write Disable Command"
      result = (written > 0) ? PARTIAL_WRITE_ERROR : WRITE_ERROR;
      goto EXIT;
    }
    startAddress += writeCount;
    written += writeCount;

#   ifdef DEBUG_WRITE
    Serial << F("\tDBG: Written = ") << written << F(", count = ") << count
	   << F(", new start address = ") << startAddress << endl;
#   endif
  }

  result = SUCCESS_RESULT;

 EXIT:
  return result;
}

uint8_t
SpiFlashMem::
eraseSector(const uint16_t sector)
{
  const uint32_t baseAddress =
    static_cast<uint32_t>(sector) * CHIP_SECTOR_SIZE;
  uint8_t result = ADDRESS_ERROR;
  if (sector > (CHIP_TOTAL_SECTORS - 1)) {
    goto EXIT;
  }
  result = beginWrite();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  sendCommand(ERASE_SECTOR_CMD);
  SPI.transfer(static_cast<uint8_t>((baseAddress >> 16) & EIGHT_BIT_MASK));
  SPI.transfer(static_cast<uint8_t>((baseAddress >> 8) & EIGHT_BIT_MASK));
  SPI.transfer(0);  // lowest order bits are ignored
  deselectChip();
  // NOTE: using timeout value from Adafruit_TinyFlash library.
  // Datasheet says 400 ms max
  result = waitForReady(10000L);
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
  // NOTE: using timeout value from Adafruit_TinyFlash library.
  // Datasheet says 6 sec max
  result = waitForReady(10000L);
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
  uint8_t result = ADDRESS_ERROR;
  if (address > (CHIP_TOTAL_BYTES - 1)) {
    goto EXIT;
  }
  result = waitForReady();
  if (SUCCESS_RESULT != result) {
    goto EXIT;
  }
  sendCommand(READ_DATA_CMD);
  SPI.transfer(static_cast<uint8_t>((address >> 16) & EIGHT_BIT_MASK));
  SPI.transfer(static_cast<uint8_t>((address >> 8) & EIGHT_BIT_MASK));
  SPI.transfer(static_cast<uint8_t>(address & EIGHT_BIT_MASK));
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
  // Send "Write Enable" command.
  sendCommand(WRITE_ENABLE_CMD);
  deselectChip();
  // Send "Read Status Register-1" command.
  sendCommand(READ_STATUS_REGISTER_1_CMD);
  result = SPI.transfer(0);
  deselectChip();
  // Check if "Write Enable Latch" (WEL) is set.
  if (!(WRITE_ENABLED_STATUS & result)) {
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
