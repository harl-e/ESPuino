#include "settings.h"

#include "RfidConfig.h"

#include "Log.h"
#include "MemX.h"
#include "System.h"

#include <CLRC663.h>
#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>
#define MFRC522_firmware_referenceV0_0
#define MFRC522_firmware_referenceV1_0
#define MFRC522_firmware_referenceV2_0
#define FM17522_firmware_reference
#include <MFRC522_I2C.h>
#include <PN5180.h>
#include <PN5180ISO14443.h>

// Global variable for the current RFID reader type
RfidReaderType activeRfidReaderType = RfidReaderType::TYPE_AUTO_DETECT; // Default to auto-detect
#if defined(RFID_READER_TYPE_RUNTIME)
// Initialize RFID reader configuration from NVS
void RfidConfig_Init(void) {
	// Try to load the RFID reader type from NVS
	activeRfidReaderType = static_cast<RfidReaderType>(gPrefsRfid.getUChar("rfidReaderType", RFID_READER_TYPE_RUNTIME)); // Default to auto-detect

	// If auto-detect is selected, try to detect the reader
	if ((activeRfidReaderType == RfidReaderType::TYPE_AUTO_DETECT) || (RfidConfig_IsReaderAvailable(activeRfidReaderType) == false)) {
		Log_Println("RFID: Auto-detecting reader type...", LOGLEVEL_INFO);
		RfidReaderType detectedType = RfidConfig_AutoDetectReader();
		if (detectedType != RfidReaderType::TYPE_AUTO_DETECT) { // valid reader detected
			activeRfidReaderType = detectedType;
			// Save the detected type to NVS for future use
			gPrefsRfid.putUChar("rfidReaderType", activeRfidReaderType);
			Log_Printf(LOGLEVEL_INFO, "RFID: Detected reader type: %d", activeRfidReaderType);
		} else {
			Log_Println("RFID: Failed to auto-detect reader type, using default", LOGLEVEL_ERROR);
			activeRfidReaderType = RfidReaderType::TYPE_PN5180; // Default to PN5180
		}
	}

	Log_Printf(LOGLEVEL_INFO, "RFID: Using reader type: %d (%s)",
		activeRfidReaderType,
		activeRfidReaderType == RfidReaderType::TYPE_MFRC522_SPI ? "MFRC522 (SPI)" : activeRfidReaderType == RfidReaderType::TYPE_MFRC522_I2C ? "MFRC522 (I2C)"
			: activeRfidReaderType == RfidReaderType::TYPE_PN5180																			  ? "PN5180"
			: activeRfidReaderType == RfidReaderType::TYPE_CLRC663_SPI													  ? "CLRC663 (SPI)"
																																			  : "Unknown");
}

// Get the current RFID reader type
RfidReaderType RfidConfig_GetReaderType(void) {
	return activeRfidReaderType;
}

// Auto-detect the RFID reader type
RfidReaderType RfidConfig_AutoDetectReader(void) {
	// start with the most common reader (PN5180) and then check for MFRC522 variants.
	if (RfidConfig_IsReaderAvailable(RfidReaderType::TYPE_PN5180)) {
		Log_Println("RFID: PN5180 reader detected", LOGLEVEL_INFO);
		return RfidReaderType::TYPE_PN5180;
	}


	if (RfidConfig_IsReaderAvailable(RfidReaderType::TYPE_CLRC663_SPI)) {
		Log_Println("RFID: CLRC663 (SPI) reader detected", LOGLEVEL_INFO);
		return RfidReaderType::TYPE_CLRC663_SPI;
	}
	if (RfidConfig_IsReaderAvailable(RfidReaderType::TYPE_MFRC522_SPI)) {
		Log_Println("RFID: MFRC522 (SPI) reader detected", LOGLEVEL_INFO);
		return RfidReaderType::TYPE_MFRC522_SPI;
	}

	if (RfidConfig_IsReaderAvailable(RfidReaderType::TYPE_MFRC522_I2C)) {
		Log_Println("RFID: MFRC522 (I2C) reader detected", LOGLEVEL_INFO);
		return RfidReaderType::TYPE_MFRC522_I2C;
	}

	Log_Println("RFID: No reader detected", LOGLEVEL_ERROR);
	return RfidReaderType::TYPE_AUTO_DETECT; // No reader detected
}

bool IsValidMfrc522Version(byte version) {
	return (
		(version == 0x82) || // Fudan Semiconductor FM17522 clone
		(version == 0x88) || // Fudan Semiconductor FM17522 clone
		(version == 0x90) || // version 0.0
		(version == 0x91) || // version 1.0
		(version == 0x92) || // version 2.0
		(version == 0xB2) || // some clones may report this
		(version == 0x12) // some clones may report this
	);
}

// Check if a specific reader type is available
bool RfidConfig_IsReaderAvailable(RfidReaderType readerType) {
	switch (readerType) {
		case RfidReaderType::TYPE_MFRC522_SPI: // MFRC522 (SPI)
			// Try to initialize MFRC522 via SPI and check if it responds
			{
				Log_Println("RFID: Checking MFRC522 (SPI)...", LOGLEVEL_DEBUG);
				MFRC522 mfrc522(RFID_CS, RST_PIN);
				SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
				mfrc522.PCD_Init();
				byte version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
				SPI.end();
				Log_Printf(LOGLEVEL_DEBUG, "RFID: MFRC522 (SPI) version=0x%02X", version);

				return (IsValidMfrc522Version(version));
			}

		case RfidReaderType::TYPE_MFRC522_I2C: // MFRC522 (I2C)
			// Try to initialize MFRC522 via I2C and check if it responds
			{
				Log_Println("RFID: Checking MFRC522 (I2C)...", LOGLEVEL_DEBUG);
				Wire.begin();
				MFRC522_I2C mfrc522(MFRC522_ADDR, RST_PIN, &Wire);
				mfrc522.PCD_Init();
				byte version = mfrc522.PCD_ReadRegister(MFRC522_I2C::VersionReg);
				Wire.end();
				Log_Printf(LOGLEVEL_DEBUG, "RFID: MFRC522 (I2C) version=0x%02X", version);

				return (IsValidMfrc522Version(version));
			}

		case RfidReaderType::TYPE_PN5180: // PN5180
			// Try to initialize PN5180 and check if it responds
			{
				Log_Println("RFID: Checking PN5180...", LOGLEVEL_DEBUG);
				PN5180 pn5180(RFID_CS, RFID_BUSY, RFID_RST);
				SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
				pn5180.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
				pn5180.reset();

				uint8_t buffer[2] = {0, 0};
				bool ok = pn5180.readEEprom(FIRMWARE_VERSION, buffer, sizeof(buffer));
				SPI.end();
				Log_Printf(LOGLEVEL_DEBUG, "RFID: PN5180 firmware read ok=%d version=%d.%d", ok ? 1 : 0, buffer[1], buffer[0]);

				// PN5180 is present if EEPROM read succeeds AND firmware version is valid.
				// Valid PN5180 versions have major version (buffer[1]) in range 1-99 (0x01-0x63) and not all 0xFF.
				// This prevents false positives from unconnected SPI bus or other devices.
				bool present = ok && (buffer[1] >= 0x01 && buffer[1] <= 0x63) && !(buffer[0] == 0xFF && buffer[1] == 0xFF);
				if (!present) {
					Log_Println("RFID: PN5180 not present or unreadable", LOGLEVEL_DEBUG);
				}
				return present;
			}

		case RfidReaderType::TYPE_CLRC663_SPI: // CLRC663
			// Try to initialize CLRC663 and check if it responds
			{
				Log_Println("RFID: Checking CLRC663...", LOGLEVEL_DEBUG);
				CLRC663 clrc663(&SPI, RFID_CS, RFID_RST);
				SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
				clrc663.begin();
				uint8_t version = clrc663.getVersion();
				SPI.end();
				Log_Printf(LOGLEVEL_DEBUG, "RFID: CLRC663 version=0x%02X", version);

				// CLRC663 is present if the version register is readable and plausible.
				// The CLRC663 family reports 0x60 (CLRC663), 0x51 (CLRC663 plus), 0x49 (CLRC66303) or
				// 0x48 (CLRC66301/02); MFRC630 reports 0x18. 0x00/0xFF indicate an unconnected or
				// floating SPI bus, which must not be mistaken for a present reader.
				bool present = (version == 0x60) || (version == 0x51) || (version == 0x49) || (version == 0x48) || (version == 0x18);
				if (!present) {
					Log_Println("RFID: CLRC663 not present or unreadable", LOGLEVEL_DEBUG);
				}
				return present;
			}

		default:
			return false;
	}
}
#endif
