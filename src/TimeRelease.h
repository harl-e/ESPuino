#pragma once

#include <Arduino.h>

#include <time.h>

// Stored as optional fields in the existing RFID NVS assignment.  No data is
// written to the physical tag.
enum class TimeReleaseIntervalUnit : uint8_t {
	Seconds,
	Months
};

struct TimeReleaseConfig {
	uint32_t startTime = 0; // Unix timestamp
	uint32_t intervalSecs = 0;
	uint32_t intervalValue = 0;
	TimeReleaseIntervalUnit intervalUnit = TimeReleaseIntervalUnit::Seconds;
};

bool TimeRelease_IsConfigValid(const TimeReleaseConfig &config);
bool TimeRelease_IsCurrentTimeValid(time_t now = 0);
// Zero-based newest released track, or -1 if there is none.
int32_t TimeRelease_GetMaxTrack(const TimeReleaseConfig &config, size_t fileCount, time_t now = 0);
