#include <Arduino.h>
#include "settings.h"

#include "RfidClrc663.h"

#include "AudioPlayer.h"
#include "HallEffectSensor.h"
#include "Log.h"
#include "MemX.h"
#include "Queues.h"
#include "Rfid.h"
#include "RfidPn5180.h"
#include "System.h"

#include <SPI.h>
#include <esp_task_wdt.h>
#include <freertos/task.h>

#if defined(RFID_READER_TYPE_RUNTIME)
	#include "RfidConfig.h"
	#include <CLRC663.h>
	#include <mfrc630_def.h>

extern unsigned long Rfid_LastRfidCheckTimestamp;
extern TaskHandle_t rfidTaskHandle;

static void RfidClrc663_Task(void *parameter);

// Chip version register (MFRC630_REG_VERSION) cached in RAM once during the reader
// task init so the management UI can display it without hardware access.
static uint8_t clrc663ChipVersion = 0;

bool RfidClrc663_GetChipVersion(uint8_t &version) {
	if (clrc663ChipVersion == 0) {
		return false;
	}
	version = clrc663ChipVersion;
	return true;
}

// Reader debounce in milliseconds: how long a previously detected card may fail
// to read before it is declared removed. Configurable via the NVS key
// "clrc663Debounce"; same semantics as the PN5180 debounce.
static uint16_t Rfid_Clrc663DebounceMs(void) {
	return gPrefsRfid.getUShort("clrc663Debounce", 500);
}

void RfidClrc663_Init(void) {
	if (rfidTaskHandle == NULL) {
		xTaskCreatePinnedToCore(
			RfidClrc663_Task, /* Function to implement the task */
			"rfid", /* Name of the task */
			3072, /* Stack size in words */
			NULL, /* Task input parameter */
			2 | portPRIVILEGE_BIT, /* Priority of the task */
			&rfidTaskHandle, /* Task handle. */
			0 /* Core where the task should run */
		);
	}
}

void RfidClrc663_Cyclic(void) {
	// Not necessary as cyclic stuff performed by task Rfid_Task()
}

void RfidClrc663_TaskReset(void) {
	Rfid_LastRfidCheckTimestamp = millis();
}

void RfidClrc663_Exit(void) {
	Log_Println("shutdown CLRC663..", LOGLEVEL_NOTICE);
	if (rfidTaskHandle != NULL) {
		vTaskDelete(rfidTaskHandle);
		rfidTaskHandle = NULL;
	}
}

void RfidClrc663_WakeupCheck(void) {
	// No deep-sleep card-detection (LPCD) support for CLRC663; wakeup falls back
	// to the button (ext0) configured in Button_Init().
}

// Prepare the reader for an ISO-14443A poll and return the UID length (0 = no card).
// Mirrors the library demo's per-poll sequence: soft reset, then load the AN1102
// recommended registers for the requested protocol before transceiving.
static uint8_t Rfid_Clrc663ReadIso14443Uid(CLRC663 &reader, uint8_t *uid) {
	reader.softReset();
	reader.AN1102_recommended_registers(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);
	return reader.read_iso14443_uid(uid);
}

// Prepare the reader for an ISO-15693 poll (incl. SLIX privacy-mode attempt) and
// return the UID length (0 = no card). The CLRC663 library stores the UID MSB
// first (uid[0] = most significant byte), while the PN5180 driver copies the
// tag's LSB-first response verbatim (PN5180ISO15693::getInventory():
// uid[i] = readBuffer[2+i], so uid[0] = E0, uid[1] = manufacturer code high, ...).
// ESPuino's card ID scheme uses the first cardIdSize bytes in PN5180 order, so
// the UID is reversed here - the same physical ISO-15693 tag then yields the
// same card ID on both readers and existing NVS assignments keep working.
static uint8_t Rfid_Clrc663ReadIso15693Uid(CLRC663 &reader, uint8_t *uid, const SlixPrivacyPassword &password) {
	reader.softReset();
	reader.AN1102_recommended_registers(MFRC630_PROTO_ISO15693_1_OF_4_SSC);
	uint8_t uidLength = reader.read_iso18693_uid(uid, const_cast<uint8_t *>(password.data()));
	if (uidLength > 1) {
		for (uint8_t i = 0; i < uidLength / 2; i++) {
			uint8_t tmp = uid[i];
			uid[i] = uid[uidLength - 1 - i];
			uid[uidLength - 1 - i] = tmp;
		}
	}
	return uidLength;
}

static void RfidClrc663_Task(void *parameter) {
// Reset pin: the CLRC663 breakout shares the PN5180 wiring on the same SPI pins, so
// RFID_RST (the PN5180 RESET line) doubles as the CLRC663's RST/NRSTPD input. RST_PIN
// is the RC522 dummy (99) and must not be used here.
	static CLRC663 reader(&SPI, RFID_CS, RFID_RST);
	SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
	reader.begin();
	clrc663ChipVersion = reader.getVersion();
	Log_Printf(LOGLEVEL_DEBUG, "CLRC663 version=0x%02X", clrc663ChipVersion);

	const uint16_t debounceMs = Rfid_Clrc663DebounceMs();
	SlixPrivacyPassword configuredSlixPrivacyPassword = SLIX_PRIVACY_PASSWORD_DEFAULT;
	if (gPrefsRfid.getBytesLength(SLIX_PRIVACY_PASSWORD_NVS_KEY) == configuredSlixPrivacyPassword.size()) {
		gPrefsRfid.getBytes(SLIX_PRIVACY_PASSWORD_NVS_KEY, configuredSlixPrivacyPassword.data(), configuredSlixPrivacyPassword.size());
	}

	byte lastValidcardId[cardIdSize] = {0}; // "same card reapplied" is decided by comparing against it
	static byte cardId[cardIdSize], lastCardId[cardIdSize];
	uint8_t uid[10];
	bool cardAppliedCurrentRun = false;
	bool cardAppliedLastRun = false;
	uint32_t lastTimeDetected = 0;
	uint8_t consecutiveMisses = 0;

	// wait until queues are created
	while (gRfidCardQueue == NULL) {
		Log_Println(waitingForTaskQueues, LOGLEVEL_DEBUG);
		vTaskDelay(50);
	}

	for (;;) {
		vTaskDelay(portTICK_PERIOD_MS * 20u);
		if (Rfid_ConsumeLastTagReset()) {
			// An assignment changed (or the web UI started playback): the card on/near the reader may now
			// mean something else, so it must not be treated as "same card re-applied" any more.
			memset(lastValidcardId, 0, sizeof(lastValidcardId));
		}

		String cardIdString;
		bool cardReceived = false;
		bool sameCardReapplied = false;

		// 1. check for an ISO-14443 card
		uint8_t uidLength = Rfid_Clrc663ReadIso14443Uid(reader, uid);
		// 2. check for an ISO-15693 card (incl. SLIX privacy mode with the configured password)
		if (uidLength == 0) {
			uidLength = Rfid_Clrc663ReadIso15693Uid(reader, uid, configuredSlixPrivacyPassword);
		}

		if (uidLength >= 4) {
			memcpy(cardId, uid, cardIdSize);
			cardReceived = true;
			consecutiveMisses = 0;
			lastTimeDetected = millis();
			cardAppliedCurrentRun = true;
		} else {
			// A card that was readable a moment ago now fails to read: either it was removed or the
			// reader response was mangled (RF noise, marginal coupling). Debounce the miss the same
			// way the PN5180 driver does, so a stationary card is not spuriously declared removed.
			if (lastTimeDetected != 0) {
				if (++consecutiveMisses == 1) {
					Log_Println("CLRC663: card read failed, debouncing", LOGLEVEL_DEBUG);
				}
				if ((millis() - lastTimeDetected) >= debounceMs) {
					lastTimeDetected = 0;
					consecutiveMisses = 0;
					cardAppliedCurrentRun = false;
					for (uint8_t i = 0; i < cardIdSize; i++) {
						lastCardId[i] = 0;
					}
				}
			} else {
				cardAppliedCurrentRun = false;
			}
		}

		if (gPlayProperties.pauseIfRfidRemoved) {
			if (!cardAppliedCurrentRun && cardAppliedLastRun) {
				Rfid_SetCardPresent(false);
				// Only pause if there's actually something to pause -- otherwise removing a card after the
				// playlist has already finished naturally queues a PAUSEPLAY that AudioPlayer_Cyclic() then
				// rejects with "no playmode change while idle", which is a confusing error for a normal action.
				if (!gPlayProperties.pausePlay && !gPlayProperties.playlistFinished && gPlayProperties.playMode != NO_PLAYLIST && System_GetOperationMode() != OPMODE_BLUETOOTH_SINK) { // Card removed => pause
					AudioPlayer_SetTrackControl(PAUSEPLAY);
					Log_Println(rfidTagRemoved, LOGLEVEL_NOTICE);
				}
			}
			cardAppliedLastRun = cardAppliedCurrentRun;
		}

		// send card to queue
		if (cardReceived) {
			// check for different card id
			if (memcmp((const void *) cardId, (const void *) lastCardId, sizeof(cardId)) == 0) {
				// Same card as last poll (the common case while a tag sits continuously on the
				// reader): skip the "new card" processing below (logging/queueing) and poll again.
				continue;
			}
			memcpy(lastCardId, cardId, cardIdSize);

#ifdef HALLEFFECT_SENSOR_ENABLE
			cardId[cardIdSize - 1] = cardId[cardIdSize - 1] + gHallEffectSensor.waitForState(HallEffectWaitMS);
#endif

			if (memcmp((const void *) lastValidcardId, (const void *) cardId, sizeof(cardId)) == 0) {
				sameCardReapplied = true;
				Log_Println("RFID same physical tag detected", LOGLEVEL_DEBUG);
			}

			String hexString;
			for (uint8_t i = 0u; i < cardIdSize; i++) {
				char str[4];
				snprintf(str, sizeof(str), "%02x%c", cardId[i], (i < cardIdSize - 1u) ? '-' : ' ');
				hexString += str;
			}
			Log_Printf(LOGLEVEL_NOTICE, rfidTagDetected, hexString.c_str());

			for (uint8_t i = 0u; i < cardIdSize; i++) {
				char num[4];
				snprintf(num, sizeof(num), "%03d", cardId[i]);
				cardIdString += num;
			}
			Rfid_SetCardPresent(true);

			if (gPlayProperties.pauseIfRfidRemoved) {
				if (!sameCardReapplied || gPlayProperties.trackFinished || gPlayProperties.playlistFinished) { // Don't allow to send card to queue if it's the same card again if track or playlist is unfnished
					xQueueSend(gRfidCardQueue, cardIdString.c_str(), 0);
				} else if (!Rfid_IsTagEligibleForResume(cardIdString.c_str())) {
					// A physical UID match alone is not enough: unknown cards and cards that no longer
					// own the active playlist must run through the normal lookup/AutoSync path again.
					Log_Printf(LOGLEVEL_DEBUG, "RFID not eligible for resume -> normal processing: %s", cardIdString.c_str());
					xQueueSend(gRfidCardQueue, cardIdString.c_str(), 0);
				} else {
					Log_Printf(LOGLEVEL_DEBUG, "RFID eligible for resume: %s", cardIdString.c_str());
					// If pause-button was pressed while card was not applied, playback could be active. If so: don't pause when card is reapplied again as the desired functionality would be reversed in this case.
					if (gPlayProperties.pausePlay && System_GetOperationMode() != OPMODE_BLUETOOTH_SINK) {
						AudioPlayer_SetTrackControl(PAUSEPLAY); // ... play/pause instead
						Log_Println(rfidTagReapplied, LOGLEVEL_NOTICE);
					}
				}
				memcpy(lastValidcardId, cardId, cardIdSize);
			} else {
				xQueueSend(gRfidCardQueue, cardIdString.c_str(), 0); // If pauseIfRfidRemoved isn't active, every card-apply leads to new playlist-generation
			}
		}
	}
}

#endif
