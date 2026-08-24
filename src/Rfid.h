#pragma once

constexpr uint8_t cardIdSize = 4u;
constexpr uint8_t cardIdStringSize = (cardIdSize * 3u) + 1u;

extern char gCurrentRfidTagId[cardIdStringSize];

// The reader tasks already determine physical presence for
// pauseIfRfidRemoved.  MediaHub AutoSync consumes this published result only
// after a completed sync; it does not poll a reader or influence downloads.
bool Rfid_IsCardPresent(void);
void Rfid_SetCardPresent(bool present);

// The RFID that successfully activated the currently loaded playlist. Reader tasks use this
// semantic ownership check before translating a same physical card into a pause/resume command.
void Rfid_MarkTagActivatedPlayback(const char *tagId);
bool Rfid_IsTagEligibleForResume(const char *tagId);

void Rfid_ResetOldRfid(void);
void Rfid_ResetLastTag(void);
// Reader-task internal: returns true once after Rfid_ResetLastTag() was called.
bool Rfid_ConsumeLastTagReset(void);
void Rfid_Init(void);
void Rfid_WakeupHandling(void);
void Rfid_StartTask(void);
void Rfid_Cyclic(void);
void Rfid_Exit(void);
void Rfid_TaskPause(void);
void Rfid_TaskResume(void);
void Rfid_TaskReset(void);
void Rfid_WakeupCheck(void);
void Rfid_PreferenceLookupHandler(void);
