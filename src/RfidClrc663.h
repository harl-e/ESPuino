#pragma once

#include <stdint.h>

// Reader-internal task, created by RfidClrc663_Init(). The CLRC663 shares the
// same SPI pins and task contract as PN5180/MFRC522 (see RfidRuntime.cpp).
void RfidClrc663_Init(void);
void RfidClrc663_Cyclic(void);
void RfidClrc663_Exit(void);
void RfidClrc663_TaskReset(void);
void RfidClrc663_WakeupCheck(void);

// Returns the CLRC663 version register value cached in RAM during the reader
// task init. No hardware access is performed here; false means no version has
// been read yet.
bool RfidClrc663_GetChipVersion(uint8_t &version);
