#include "TimeRelease.h"

#include <time.h>

bool TimeRelease_IsConfigValid(const TimeReleaseConfig &config) {
	return config.startTime > 0 && (config.intervalUnit == TimeReleaseIntervalUnit::Months ? config.intervalValue > 0 : config.intervalSecs > 0);
}

bool TimeRelease_IsCurrentTimeValid(time_t now) {
	if (now == 0) {
		now = time(nullptr);
	}
	// The established NTP/configTzTime clock owns this value.  The ESP epoch
	// must never unlock content before a trustworthy synchronization.
	return now >= 1704067200; // 2024-01-01 UTC
}

static time_t TimeRelease_MonthReleaseTime(const TimeReleaseConfig &config, uint64_t releaseIndex) {
	tm startLocal {};
	const time_t start = static_cast<time_t>(config.startTime);
	localtime_r(&start, &startLocal);
	const uint64_t monthsAfterStart = releaseIndex * config.intervalValue;
	tm target = startLocal;
	target.tm_year += static_cast<int>((startLocal.tm_mon + monthsAfterStart) / 12);
	target.tm_mon = static_cast<int>((startLocal.tm_mon + monthsAfterStart) % 12);
	target.tm_mday = 1;
	target.tm_isdst = -1;
	tm monthEnd = target;
	monthEnd.tm_mon++;
	monthEnd.tm_mday = 0;
	mktime(&monthEnd);
	target.tm_mday = min(startLocal.tm_mday, monthEnd.tm_mday);
	return mktime(&target);
}

int32_t TimeRelease_GetMaxTrack(const TimeReleaseConfig &config, size_t fileCount, time_t now) {
	if (!TimeRelease_IsConfigValid(config) || fileCount == 0) {
		return -1;
	}
	if (now == 0) {
		now = time(nullptr);
	}
	if (!TimeRelease_IsCurrentTimeValid(now) || now < static_cast<time_t>(config.startTime)) {
		return -1;
	}
	if (config.intervalUnit == TimeReleaseIntervalUnit::Seconds) {
		const uint64_t releaseIndex = static_cast<uint64_t>(now - static_cast<time_t>(config.startTime)) / config.intervalSecs;
		return static_cast<int32_t>(releaseIndex >= fileCount ? fileCount - 1 : releaseIndex);
	}
	size_t low = 0;
	size_t high = fileCount - 1;
	while (low < high) {
		const size_t middle = low + (high - low + 1) / 2;
		if (TimeRelease_MonthReleaseTime(config, middle) <= now) low = middle;
		else high = middle - 1;
	}
	return static_cast<int32_t>(low);
}
