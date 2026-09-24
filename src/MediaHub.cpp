#include <Arduino.h>
#include "settings.h"

#include "MediaHub.h"

#include "AudioPlayer.h"
#include "Common.h"
#include "Led.h"
#include "Log.h"
#include "Rfid.h"
#include "SdCard.h"
#include "System.h"
#include "Wlan.h"
#include "logmessages.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <mbedtls/sha256.h>
#include <nvs.h>

const char *const MediaHub_PathPrefix = "mediahub://";

// NVS keys are new and intentionally default to false when absent.  Existing
// installations must retain their exact pre-AutoSync behaviour after update.
static constexpr char MediaHub_AutoSyncNvsKey[] = "mhAutoSync";
static constexpr char MediaHub_VisibleStorageNvsKey[] = "mhVisible";
static_assert(sizeof(MediaHub_AutoSyncNvsKey) - 1 <= 15, "NVS key must fit in 15 characters");
static_assert(sizeof(MediaHub_VisibleStorageNvsKey) - 1 <= 15, "NVS key must fit in 15 characters");

// Short on purpose (concept §14): an unreachable hub must never turn into
// a long hang while a card is being tapped.
static constexpr int32_t MediaHub_ConnectTimeoutMs = 2000;
static constexpr uint16_t MediaHub_ReadTimeoutMs = 3000;

// Per-chunk stall timeout while downloading a file body: media files can be
// large, so this is a "no bytes at all for this long" watchdog, not a total
// transfer-time budget.
static constexpr uint32_t MediaHub_DownloadStallTimeoutMs = 8000;
// Matches start_chunk_size in Web.cpp's upload path: bigger chunks mean fewer
// read()/write() round-trips per file, which is where most of the throughput
// difference against the (~450 kB/s) upload path came from besides power-save.
static constexpr size_t MediaHub_DownloadBufferSize = 16384;

// Set for the duration of an actual file download (concept §14/#16): a card
// tapped while this is true gets a "busy" error instead of being processed.
// Atomic: the check-then-act gap of a plain volatile flag let the web task's
// cleanup race a concurrent card tap (TOCTOU) across cores.
static std::atomic<bool> MediaHub_DownloadBusy {false};
// Explicit sync has short non-download phases as well.  Maintenance must not
// race its manifest writes, so it has a separate, equally small busy marker.
static std::atomic<bool> MediaHub_SyncBusy {false};

struct MediaHub_BusyGuard {
	MediaHub_BusyGuard() {
		MediaHub_DownloadBusy.store(true, std::memory_order_relaxed);
		// Full WiFi power for the duration of the download - same win the
		// upload path gets from System_PauseTasksDuringUpload(). Deliberately
		// not reusing that function: it also suspends Led_Task and the RFID
		// task, which would kill the non-suspending download animation from
		// Phase 5 and pause the very task this download is running on.
		Wlan_SetPowerSave(false);
	}
	~MediaHub_BusyGuard() {
		MediaHub_DownloadBusy.store(false, std::memory_order_relaxed);
		Wlan_SetPowerSave(true);
		Led_SetDownloadProgress(false);
	}
};

// Fixed, hidden storage layout (concept §13.1) — MediaHub is the only thing
// that ever touches this tree.
static String MediaHub_BuildBaseUrl(const String &hostPort);

static String MediaHub_HubKey(const String &hostPort) {
	const String baseUrl = MediaHub_BuildBaseUrl(hostPort);
	uint8_t digest[32];
	mbedtls_sha256_context context;
	mbedtls_sha256_init(&context);
	mbedtls_sha256_starts(&context, 0);
	mbedtls_sha256_update(&context, reinterpret_cast<const unsigned char *>(baseUrl.c_str()), baseUrl.length());
	mbedtls_sha256_finish(&context, digest);
	mbedtls_sha256_free(&context);
	static constexpr char hex[] = "0123456789abcdef";
	String key;
	key.reserve(16);
	for (size_t index = 0; index < 8; ++index) {
		key += hex[digest[index] >> 4];
		key += hex[digest[index] & 0x0F];
	}
	return key;
}

String MediaHub_GetHubKey(const String &hostPort) {
	return MediaHub_HubKey(hostPort);
}

static String MediaHub_ManifestCachePath(const char *cardId, const String &hostPort = "") {
	if (MediaHub_UseVisibleStorage() && hostPort.length() > 0) {
		return "/.mediahub/manifests/" + MediaHub_HubKey(hostPort) + "/" + String(cardId) + ".json";
	}
	return "/.mediahub/manifests/" + String(cardId) + ".json";
}

static String MediaHub_VisibleHubDirectory(const String &hostPort) {
	const String hubKey = MediaHub_HubKey(hostPort);
	for (const MediaHubServer &server : MediaHub_GetServers()) {
		const String configuredHost = String(server.https ? "https://" : "http://") + server.hostPort;
		if (MediaHub_HubKey(configuredHost) == hubKey && server.alias.length() > 0) {
			return server.alias;
		}
	}
	return hubKey;
}

static String MediaHub_MediaDir(const char *cardId, const String &hostPort = "") {
	if (MediaHub_UseVisibleStorage() && hostPort.length() > 0) {
		// The stable hub key remains the technical identity of a MediaHub.  A
		// user-defined alias is used only for future visible-storage paths.
		// Renaming the alias intentionally does not migrate existing folders.
		return "/MediaHub/" + MediaHub_VisibleHubDirectory(hostPort);
	}
	return "/.mediahub/media/" + String(cardId);
}

// The NVS path field's "hostPort" part may itself already carry a scheme
// (e.g. "https://myhub:8443", written there via the Web-UI's http/https
// dropdown, concept §5.1) - use it as-is if so, otherwise default to plain
// http:// for the common case. HTTPClient::begin() already handles either
// scheme transparently (falling back to an unverified/insecure TLS
// connection for https - fine for a local, trusted hub, concept #19).
static String MediaHub_BuildBaseUrl(const String &hostPort) {
	if (hostPort.startsWith("http://") || hostPort.startsWith("https://")) {
		return hostPort;
	}
	return "http://" + hostPort;
}

// "stale"/"needs resync" (concept §9/§13) share one marker and one recovery
// path: both mean "the next tap should wipe and fully re-download". A
// re-sync that fails partway simply never clears the marker, which is
// exactly the "needs resync" behavior the concept describes.
static String MediaHub_StaleMarkerPath(const char *cardId, const String &hostPort = "") {
	return MediaHub_ManifestCachePath(cardId, hostPort) + ".stale";
}

// True if the stale marker exists.
static bool MediaHub_IsStale(const char *cardId, const String &hostPort = "") {
	return gFSystem.exists(MediaHub_StaleMarkerPath(cardId, hostPort));
}

// Creates the stale marker.
static void MediaHub_MarkStale(const char *cardId, const String &hostPort = "") {
	File f = gFSystem.open(MediaHub_StaleMarkerPath(cardId, hostPort), FILE_WRITE, true);
	if (f) {
		f.close();
	}
}

// Removes the stale marker.
static void MediaHub_ClearStale(const char *cardId, const String &hostPort = "") {
	gFSystem.remove(MediaHub_StaleMarkerPath(cardId, hostPort));
}

// SINGLE_TRACK/SINGLE_TRACK_LOOP name one specific file, not a folder to
// scan (mirrors SINGLE_FILE_PLAY_MODES on the hub, which caps assignment
// to exactly one file for these two modes).
static bool MediaHub_IsSingleFilePlayMode(uint32_t playMode) {
	return playMode == SINGLE_TRACK || playMode == SINGLE_TRACK_LOOP;
}

static bool MediaHub_GetTimeReleaseConfig(JsonVariantConst manifest, uint32_t playMode, TimeReleaseConfig &config) {
	if (playMode != TIME_RELEASE) {
		return true;
	}
	config.startTime = manifest["timeReleaseStart"] | 0;
	config.intervalSecs = manifest["timeReleaseInterval"] | 0;
	const char *intervalUnit = manifest["timeReleaseIntervalUnit"] | "";
	if (strcmp(intervalUnit, "months") == 0) {
		config.intervalUnit = TimeReleaseIntervalUnit::Months;
		config.intervalValue = manifest["timeReleaseIntervalValue"] | 0;
	}
	if (!TimeRelease_IsConfigValid(config)) {
		Log_Println("MediaHub: TIME_RELEASE manifest has invalid timing", LOGLEVEL_ERROR);
		return false;
	}
	return true;
}

static String MediaHub_Sha256Hex(const uint8_t digest[32]);

// Local integrity check is size-only, never a hash (concept §9): hashing a
// possibly 100 MB file on every tap just to confirm what the hub already
// verified at download time isn't an option on this hardware.
static bool MediaHub_FileFullySynced(const String &mediaDir, JsonVariantConst fileEntry) {
	const char *path = fileEntry["path"] | "";
	if (strlen(path) == 0) {
		return false;
	}
	const uint32_t expectedSize = fileEntry["size"] | 0;
	File f = gFSystem.open(mediaDir + "/" + path);
	if (!f || f.isDirectory()) {
		return false;
	}
	const bool sizeMatches = (uint32_t) f.size() == expectedSize;
	f.close();
	return sizeMatches;
}

static bool MediaHub_FileIdentityMatches(JsonArrayConst cachedFiles, JsonVariantConst fileEntry) {
	const char *path = fileEntry["path"] | "";
	const char *sha256 = fileEntry["sha256"] | "";
	const uint32_t size = fileEntry["size"] | 0;
	for (JsonVariantConst cached : cachedFiles) {
		if (strcmp(cached["path"] | "", path) == 0 && (uint32_t) (cached["size"] | 0) == size
			&& strcmp(cached["sha256"] | "", sha256) == 0) {
			return true;
		}
	}
	return false;
}

static bool MediaHub_FileHashMatches(const String &path, const char *expectedSha256) {
	if (expectedSha256 == nullptr || strlen(expectedSha256) != 64) {
		return false;
	}
	File file = gFSystem.open(path);
	if (!file || file.isDirectory()) {
		file.close();
		return false;
	}
	mbedtls_sha256_context context;
	mbedtls_sha256_init(&context);
	mbedtls_sha256_starts(&context, 0);
	uint8_t buffer[1024];
	while (file.available()) {
		const size_t read = file.read(buffer, sizeof(buffer));
		if (read == 0) {
			file.close();
			mbedtls_sha256_free(&context);
			return false;
		}
		mbedtls_sha256_update(&context, buffer, read);
		esp_task_wdt_reset();
	}
	file.close();
	uint8_t digest[32];
	mbedtls_sha256_finish(&context, digest);
	mbedtls_sha256_free(&context);
	return MediaHub_Sha256Hex(digest).equalsIgnoreCase(expectedSha256);
}

static bool MediaHub_FileValidForManifest(const String &mediaDir, JsonArrayConst cachedFiles, JsonVariantConst fileEntry) {
	if (!MediaHub_FileFullySynced(mediaDir, fileEntry)) {
		return false;
	}
	if (MediaHub_FileIdentityMatches(cachedFiles, fileEntry)) {
		return true;
	}
	const char *path = fileEntry["path"] | "";
	return MediaHub_FileHashMatches(mediaDir + "/" + path, fileEntry["sha256"] | "");
}

// True only if every file in `files` passes MediaHub_FileFullySynced().
static bool MediaHub_AllFilesSynced(const String &mediaDir, JsonArrayConst files) {
	if (files.size() == 0) {
		return false; // nothing to play
	}
	for (JsonVariantConst f : files) {
		if (!MediaHub_FileFullySynced(mediaDir, f)) {
			return false;
		}
	}
	return true;
}

// Directory portion of a manifest path, or "" if it sits at the library root.
static String MediaHub_DirOf(const String &path) {
	const int slash = path.lastIndexOf('/');
	if (slash < 0) {
		return "";
	}
	return path.substring(0, slash);
}

// Splits a path into its '/'-separated segments.
static std::vector<String> MediaHub_SplitPath(const String &path) {
	std::vector<String> segments;
	int start = 0;
	while (start <= (int) path.length()) {
		int slash = path.indexOf('/', start);
		if (slash < 0) {
			segments.push_back(path.substring(start));
			break;
		}
		segments.push_back(path.substring(start, slash));
		start = slash + 1;
	}
	return segments;
}

// Longest common leading path-segments across every file's directory.
// Non-single-file manifests are always built from one selected folder on the
// hub, optionally scanned recursively (concept §5.4) - this reliably yields
// exactly that folder back, regardless of how deep individual files end up
// nested under it (which AudioPlayer_SetPlaylist()'s own recursion, keyed
// off the real playMode, then handles from there). Returns "" if the files
// sit directly at the library root, i.e. mediaDir is already the right item.
static String MediaHub_CommonDirectory(JsonArrayConst files) {
	std::vector<String> common;
	bool first = true;
	for (JsonVariantConst f : files) {
		const char *path = f["path"] | "";
		const String dir = MediaHub_DirOf(String(path));
		std::vector<String> segments = dir.length() > 0 ? MediaHub_SplitPath(dir) : std::vector<String>();
		if (first) {
			common = segments;
			first = false;
			continue;
		}
		size_t matched = 0;
		while (matched < common.size() && matched < segments.size() && common[matched] == segments[matched]) {
			matched++;
		}
		common.resize(matched);
	}
	String result;
	for (size_t i = 0; i < common.size(); i++) {
		if (i > 0) {
			result += "/";
		}
		result += common[i];
	}
	return result;
}

// SINGLE_TRACK/SINGLE_TRACK_LOOP point at one specific file. Every other mode
// points at the folder the files actually share (see MediaHub_CommonDirectory())
// and leaves scanning/sorting/recursion from there to SdCard_ReturnPlaylist()
// (same reasoning as in AudioPlayer_SetPlaylist()) - pointing at mediaDir
// itself would be wrong whenever files[].path carries subdirectories
// (recursive selections, or any folder that isn't the media library root).
static String MediaHub_BuildItemToPlay(const String &mediaDir, uint32_t playMode, JsonArrayConst files) {
	if (MediaHub_IsSingleFilePlayMode(playMode)) {
		const char *singleFilePath = files[0]["path"] | "";
		return mediaDir + "/" + singleFilePath;
	}
	const String commonDir = MediaHub_CommonDirectory(files);
	if (commonDir.length() == 0) {
		return mediaDir;
	}
	return mediaDir + "/" + commonDir;
}

// Mirrors explorerDeleteDirectory() in Web.cpp: recurse via File objects only,
// never rebuild string paths for entries, so SanitizedFS's percent-encoding
// (FileSystem.h) can't be applied twice to an already-sanitized name.
static bool MediaHub_DeleteDirRecursive(File dir) {
	bool ok = true;
	File entry = dir.openNextFile();
	while (entry) {
		if (entry.isDirectory()) {
			ok &= MediaHub_DeleteDirRecursive(entry);
		} else {
			ok &= gFSystem.remove(entry);
		}
		entry = dir.openNextFile();
		esp_task_wdt_reset();
	}
	return gFSystem.rmdir(dir) && ok;
}

// Cleanup is intentionally restricted to the hidden MediaHub cache. Visible
// /MediaHub content is user-managed and must never be deleted here.
static bool MediaHub_IsHiddenCleanupPath(const String &path) {
	static constexpr char manifestsRoot[] = "/.mediahub/manifests";
	static constexpr char mediaRoot[] = "/.mediahub/media";
	if (path.length() == 0 || path == "/" || path == "." || path.indexOf("..") >= 0) {
		return false;
	}
	return path == manifestsRoot || path == mediaRoot || path.startsWith(String(manifestsRoot) + "/")
		|| path.startsWith(String(mediaRoot) + "/");
}

static bool MediaHub_DeleteHiddenDir(const String &path) {
	if (!MediaHub_IsHiddenCleanupPath(path)) {
		Log_Printf(LOGLEVEL_ERROR, "MediaHub cleanup: refused unsafe path: %s", path.c_str());
		return false;
	}
	File dir = gFSystem.open(path);
	if (!dir || !dir.isDirectory()) {
		return true; // an already absent cache is clean
	}
	return MediaHub_DeleteDirRecursive(dir);
}

static bool MediaHub_RemoveHiddenFile(const String &path) {
	if (!MediaHub_IsHiddenCleanupPath(path)) {
		Log_Printf(LOGLEVEL_ERROR, "MediaHub cleanup: refused unsafe path: %s", path.c_str());
		return false;
	}
	return !gFSystem.exists(path) || gFSystem.remove(path);
}

// Same File-object-based recursion as MediaHub_DeleteDirRecursive(), for the
// same reason: avoids re-sanitizing an already-sanitized on-disk path.
static uint64_t MediaHub_DirSizeRecursive(File dir) {
	uint64_t total = 0;
	File entry = dir.openNextFile();
	while (entry) {
		if (entry.isDirectory()) {
			total += MediaHub_DirSizeRecursive(entry);
		} else {
			total += entry.size();
		}
		entry = dir.openNextFile();
		esp_task_wdt_reset();
	}
	return total;
}

// Total size of an existing directory's contents, 0 if it doesn't exist.
static uint64_t MediaHub_DirSize(const String &path) {
	File dir = gFSystem.open(path);
	if (!dir || !dir.isDirectory()) {
		return 0;
	}
	return MediaHub_DirSizeRecursive(dir);
}

// Percent-encodes a manifest file path for use in the download URL. Library
// paths come straight from the hub's filesystem and routinely contain
// spaces or other characters that aren't valid unencoded in a URL (distinct
// from SanitizedFS's FAT-illegal-char encoding used for local storage,
// which is unrelated and already handled elsewhere). '/' is preserved since
// it's the path separator, not data to encode.
static String MediaHub_UrlEncodePath(const String &path) {
	String encoded;
	encoded.reserve(path.length());
	for (size_t i = 0; i < path.length(); i++) {
		const uint8_t c = (uint8_t) path[i];
		if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~' || c == '/') {
			encoded += (char) c;
		} else {
			char hex[4];
			snprintf(hex, sizeof(hex), "%%%02X", c);
			encoded += hex;
		}
	}
	return encoded;
}

static String MediaHub_Sha256Hex(const uint8_t digest[32]) {
	static const char *hexDigits = "0123456789abcdef";
	String hex;
	hex.reserve(64);
	for (int i = 0; i < 32; i++) {
		hex += hexDigits[digest[i] >> 4];
		hex += hexDigits[digest[i] & 0x0F];
	}
	return hex;
}

// Double-buffered download/write, mirroring explorerHandleFileUpload() /
// explorerHandleFileStorageTask() in Web.cpp: while one buffer is being
// written to SD by a dedicated task, the network read loop already fills the
// other one, instead of serializing "read network" and "write SD" on the same
// task.
//
// Allocated fresh per file, *after* that file's TLS handshake already
// succeeded, and freed again once the transfer is done - not as a permanent
// compile-time internal-DRAM array. A TLS handshake needs ~32-40 KB of
// *internal* heap for its fixed in/out buffers (CONFIG_MBEDTLS_SSL_VARIABLE_
// BUFFER_LENGTH is off); parking these 32 KB permanently in internal DRAM
// left too little of it free/contiguous for that handshake to succeed on
// https downloads (seen as HTTPC_ERROR_CONNECTION_REFUSED on real hardware).
// Internal RAM stays the preferred allocator (PSRAM is SPI-attached and
// measurably slower - moving these buffers there permanently cost the
// throughput this double-buffering exists for in the first place); PSRAM is
// only a last-resort fallback if internal allocation fails even at this
// later, safer point.
static constexpr size_t MediaHub_DownloadNumBuffers = 2;
static uint8_t *MediaHub_DownloadBuffers[MediaHub_DownloadNumBuffers] = {nullptr, nullptr};
static std::atomic<uint32_t> MediaHub_DownloadBufferBytes[MediaHub_DownloadNumBuffers];
static std::atomic<bool> MediaHub_DownloadBufferFull[MediaHub_DownloadNumBuffers];
static TaskHandle_t MediaHub_DownloadWriterTaskHandle = NULL;
static SemaphoreHandle_t MediaHub_DownloadWriterDone = NULL;
static volatile bool MediaHub_DownloadWriterError = false;

enum MediaHub_DownloadChunkHistogramBucket : uint8_t {
	MediaHub_DownloadChunk1To127,
	MediaHub_DownloadChunk128To511,
	MediaHub_DownloadChunk512To1023,
	MediaHub_DownloadChunk1024To4095,
	MediaHub_DownloadChunk4096To8191,
	MediaHub_DownloadChunk8192OrMore,
	MediaHub_DownloadChunkHistogramBucketCount,
};

struct MediaHub_SyncBusyGuard {
	MediaHub_SyncBusyGuard() { MediaHub_SyncBusy.store(true, std::memory_order_relaxed); }
	~MediaHub_SyncBusyGuard() { MediaHub_SyncBusy.store(false, std::memory_order_relaxed); }
};

// Per-file diagnostics only. The producer owns HTTP/hash fields and the
// writer task owns SD/close fields; the producer reads the latter only after
// MediaHub_DownloadWriterDone has been given.
struct MediaHub_DownloadDiagnostics {
	uint64_t httpReadTimeUs = 0;
	uint64_t httpBytes = 0;
	uint32_t httpReadCalls = 0;
	uint32_t httpMaxChunk = 0;
	uint32_t httpChunkHistogram[MediaHub_DownloadChunkHistogramBucketCount] = {};
	uint64_t sdWriteTimeUs = 0;
	uint64_t sdBytes = 0;
	uint32_t sdWriteCalls = 0;
	uint32_t sdMaxChunk = 0;
	uint32_t sdChunkHistogram[MediaHub_DownloadChunkHistogramBucketCount] = {};
	uint64_t hashTimeUs = 0;
	uint64_t closeTimeUs = 0;
	uint64_t renameTimeUs = 0;
	uint64_t noDataWaitTimeUs = 0;
	uint64_t writerWaitTimeUs = 0;
};

static void MediaHub_RecordDownloadChunk(uint32_t histogram[MediaHub_DownloadChunkHistogramBucketCount], size_t bytes) {
	if (bytes < 128) {
		++histogram[MediaHub_DownloadChunk1To127];
	} else if (bytes < 512) {
		++histogram[MediaHub_DownloadChunk128To511];
	} else if (bytes < 1024) {
		++histogram[MediaHub_DownloadChunk512To1023];
	} else if (bytes < 4096) {
		++histogram[MediaHub_DownloadChunk1024To4095];
	} else if (bytes < 8192) {
		++histogram[MediaHub_DownloadChunk4096To8191];
	} else {
		++histogram[MediaHub_DownloadChunk8192OrMore];
	}
}

static void MediaHub_LogDownloadDiagnostics(const MediaHub_DownloadDiagnostics &diagnostics, uint64_t bytesTotal,
	uint64_t durationTotalUs) {
	const float rateKBps = durationTotalUs > 0 ? (bytesTotal * 1000000.0f) / (1024.0f * durationTotalUs) : 0.0f;
	// HTTP reads and SD writes run in parallel. Therefore the timed fields are
	// intentionally not additive; other_time_us is wall time outside the
	// producer's readBytes()/SHA-256-update() calls and includes coordination.
	const uint64_t producerMeasuredUs = diagnostics.httpReadTimeUs + diagnostics.hashTimeUs;
	const uint64_t otherTimeUs = durationTotalUs > producerMeasuredUs ? durationTotalUs - producerMeasuredUs : 0;
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: bytes_total=%llu duration_total_ms=%llu effective_rate_kBps=%.2f",
		(unsigned long long) bytesTotal, (unsigned long long) (durationTotalUs / 1000), rateKBps);
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: http_read_calls=%u http_bytes=%llu http_read_time_us=%llu http_avg_chunk=%llu http_max_chunk=%u",
		diagnostics.httpReadCalls, (unsigned long long) diagnostics.httpBytes, (unsigned long long) diagnostics.httpReadTimeUs,
		(unsigned long long) (diagnostics.httpReadCalls > 0 ? diagnostics.httpBytes / diagnostics.httpReadCalls : 0), diagnostics.httpMaxChunk);
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: sd_write_calls=%u sd_bytes=%llu sd_write_time_us=%llu sd_avg_chunk=%llu sd_max_chunk=%u",
		diagnostics.sdWriteCalls, (unsigned long long) diagnostics.sdBytes, (unsigned long long) diagnostics.sdWriteTimeUs,
		(unsigned long long) (diagnostics.sdWriteCalls > 0 ? diagnostics.sdBytes / diagnostics.sdWriteCalls : 0), diagnostics.sdMaxChunk);
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: hash_time_us=%llu flush_time_us=0 close_time_us=%llu rename_time_us=%llu other_time_us=%llu",
		(unsigned long long) diagnostics.hashTimeUs, (unsigned long long) diagnostics.closeTimeUs,
		(unsigned long long) diagnostics.renameTimeUs, (unsigned long long) otherTimeUs);
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: no_data_wait_time_us=%llu writer_wait_time_us=%llu",
		(unsigned long long) diagnostics.noDataWaitTimeUs, (unsigned long long) diagnostics.writerWaitTimeUs);
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: http_chunk_hist=1-127:%u 128-511:%u 512-1023:%u 1024-4095:%u 4096-8191:%u >=8192:%u",
		diagnostics.httpChunkHistogram[MediaHub_DownloadChunk1To127], diagnostics.httpChunkHistogram[MediaHub_DownloadChunk128To511],
		diagnostics.httpChunkHistogram[MediaHub_DownloadChunk512To1023], diagnostics.httpChunkHistogram[MediaHub_DownloadChunk1024To4095],
		diagnostics.httpChunkHistogram[MediaHub_DownloadChunk4096To8191], diagnostics.httpChunkHistogram[MediaHub_DownloadChunk8192OrMore]);
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub download diagnostics: sd_chunk_hist=1-127:%u 128-511:%u 512-1023:%u 1024-4095:%u 4096-8191:%u >=8192:%u",
		diagnostics.sdChunkHistogram[MediaHub_DownloadChunk1To127], diagnostics.sdChunkHistogram[MediaHub_DownloadChunk128To511],
		diagnostics.sdChunkHistogram[MediaHub_DownloadChunk512To1023], diagnostics.sdChunkHistogram[MediaHub_DownloadChunk1024To4095],
		diagnostics.sdChunkHistogram[MediaHub_DownloadChunk4096To8191], diagnostics.sdChunkHistogram[MediaHub_DownloadChunk8192OrMore]);
}

static bool MediaHub_EnsureDownloadBuffersAllocated() {
	for (size_t i = 0; i < MediaHub_DownloadNumBuffers; i++) {
		if (MediaHub_DownloadBuffers[i] != nullptr) {
			continue;
		}
		MediaHub_DownloadBuffers[i] = static_cast<uint8_t *>(heap_caps_malloc(MediaHub_DownloadBufferSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		if (MediaHub_DownloadBuffers[i] == nullptr) {
			MediaHub_DownloadBuffers[i] = static_cast<uint8_t *>(heap_caps_malloc(MediaHub_DownloadBufferSize, MALLOC_CAP_SPIRAM));
		}
		if (MediaHub_DownloadBuffers[i] == nullptr) {
			return false;
		}
	}
	return true;
}

// Releases the two download buffers again once a file's transfer is done, so
// they aren't parked in internal RAM across the next file's TLS handshake.
static void MediaHub_FreeDownloadBuffers() {
	for (size_t i = 0; i < MediaHub_DownloadNumBuffers; i++) {
		heap_caps_free(MediaHub_DownloadBuffers[i]);
		MediaHub_DownloadBuffers[i] = nullptr;
	}
}

struct MediaHub_WriterTaskArgs {
	File file; // already open at index 0, created by the caller
	MediaHub_DownloadDiagnostics *diagnostics;
};

// Drains full buffers to `file` as they become available. Notification value
// 1 = producer done normally (drain what's left, then exit); 2 = producer
// aborted (exit immediately, whatever's unwritten doesn't matter since the
// caller discards the .part file on any error anyway).
static void MediaHub_DownloadWriterTask(void *parameter) {
	auto *args = static_cast<MediaHub_WriterTaskArgs *>(parameter);
	File file = args->file;
	MediaHub_DownloadDiagnostics *diagnostics = args->diagnostics;
	delete args;

	uint32_t readIndex = 0;
	for (;;) {
		uint32_t notifyValue = 0;
		const BaseType_t notified = xTaskNotifyWait(0, 0, &notifyValue, 0);
		if (notified == pdPASS && notifyValue == 2u) {
			break; // producer aborted, don't bother writing anything more
		}
		if (MediaHub_DownloadBufferFull[readIndex]) {
			while (MediaHub_DownloadBufferFull[readIndex]) {
				const uint32_t len = MediaHub_DownloadBufferBytes[readIndex];
				if (len > 0) {
					const int64_t writeStartUs = esp_timer_get_time();
					const size_t written = file.write(MediaHub_DownloadBuffers[readIndex], len);
					diagnostics->sdWriteTimeUs += esp_timer_get_time() - writeStartUs;
					diagnostics->sdBytes += written;
					++diagnostics->sdWriteCalls;
					diagnostics->sdMaxChunk = std::max(diagnostics->sdMaxChunk, (uint32_t) written);
					MediaHub_RecordDownloadChunk(diagnostics->sdChunkHistogram, written);
					if (written != len) {
						MediaHub_DownloadWriterError = true;
					}
				}
				MediaHub_DownloadBufferBytes[readIndex] = 0;
				MediaHub_DownloadBufferFull[readIndex] = false;
				readIndex = (readIndex + 1) % MediaHub_DownloadNumBuffers;
				if (MediaHub_DownloadWriterError) {
					break; // don't keep writing to a file that just failed
				}
				esp_task_wdt_reset();
			}
			if (MediaHub_DownloadWriterError || (notified == pdPASS && notifyValue == 1u)) {
				break; // write failed, or that was the last (partial) buffer
			}
		} else if (notified == pdPASS && notifyValue == 1u) {
			break; // done, and nothing was left to drain
		} else {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}
	const int64_t closeStartUs = esp_timer_get_time();
	file.close();
	diagnostics->closeTimeUs += esp_timer_get_time() - closeStartUs;
	MediaHub_DownloadWriterTaskHandle = NULL;
	xSemaphoreGive(MediaHub_DownloadWriterDone);
	vTaskDelete(NULL);
}

// Downloads one file to <finalPath>.part, hashing incrementally while writing
// (concept §9: SHA-256 only ever checked during download, never recomputed
// over local files afterwards). Renames into place only once size and hash
// both match; leaves no partial/renamed file behind on any failure.
static bool MediaHub_DownloadAndVerifyFile(const String &fileUrl, const String &finalPath, uint32_t expectedSize, const char *expectedSha256Hex, uint64_t &completedBytes, uint64_t totalBytes) {
	const int64_t downloadStartUs = esp_timer_get_time();
	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_DownloadStallTimeoutMs);
	if (!http.begin(fileUrl)) {
		Log_Printf(LOGLEVEL_ERROR, mediaHubDownloadFailed, fileUrl.c_str());
		return false;
	}

	// The handshake (connect + TLS) happens here, before the download buffers
	// exist - so it never has to compete with them for internal heap.
	const int httpCode = http.GET();
	if (httpCode != HTTP_CODE_OK) {
		Log_Printf(LOGLEVEL_ERROR, mediaHubDownloadHttpError, httpCode, fileUrl.c_str());
		http.end();
		return false;
	}

	if (!MediaHub_EnsureDownloadBuffersAllocated()) {
		Log_Printf(LOGLEVEL_ERROR, mediaHubDownloadFailed, fileUrl.c_str());
		http.end();
		return false;
	}

	const String tmpPath = finalPath + ".part";
	// A previous failed transfer leaves a discarded part file behind.  Start
	// every retry from zero instead of relying on filesystem open semantics
	// (which may append for FILE_WRITE on some backends).
	gFSystem.remove(tmpPath);
	File tmpFile = gFSystem.open(tmpPath, FILE_WRITE, true); // create=true: also creates missing parent dirs
	if (!tmpFile) {
		Log_Printf(LOGLEVEL_ERROR, mediaHubDownloadFileError, fileUrl.c_str());
		http.end();
		MediaHub_FreeDownloadBuffers();
		return false;
	}

	mbedtls_sha256_context shaCtx;
	mbedtls_sha256_init(&shaCtx);
	mbedtls_sha256_starts(&shaCtx, 0); // 0 = SHA-256 (not the SHA-224 variant)

	// Hand the open file off to the writer task; from here on the producer
	// (this function/task) only ever touches the buffers, never the file.
	for (size_t i = 0; i < MediaHub_DownloadNumBuffers; i++) {
		MediaHub_DownloadBufferBytes[i] = 0;
		MediaHub_DownloadBufferFull[i] = false;
	}
	MediaHub_DownloadWriterError = false;
	if (MediaHub_DownloadWriterDone == NULL) {
		MediaHub_DownloadWriterDone = xSemaphoreCreateBinary();
	} else {
		xSemaphoreTake(MediaHub_DownloadWriterDone, 0); // make sure it's empty before this run
	}
	MediaHub_DownloadDiagnostics diagnostics;
	auto *writerArgs = new MediaHub_WriterTaskArgs {tmpFile, &diagnostics};
	xTaskCreatePinnedToCore(MediaHub_DownloadWriterTask, "MediaHubWriter", 3072, writerArgs, 2, &MediaHub_DownloadWriterTaskHandle, 1);

	auto *stream = http.getStreamPtr();
	size_t totalRead = 0;
	uint32_t writeIndex = 0;
	bool transferError = false;
	bool restartRequested = false;
	const uint32_t transferStartMs = millis();
	uint32_t lastDataMs = transferStartMs;

	while (totalRead < expectedSize) {
		// A pending restart/shutdown must never wait out a multi-minute download
		// first (concept-adjacent hardening): bail out now, System_Cyclic() picks
		// it up as soon as this call unwinds back to loop(). Whatever's on disk
		// stays a discarded .part file either way, so there's nothing to protect.
		if (System_IsRestartOrSleepPending()) {
			restartRequested = true;
			break;
		}
		if (MediaHub_DownloadWriterError) {
			transferError = true;
			break;
		}
		const int avail = stream->available();
		if (avail <= 0) {
			if (millis() - lastDataMs > MediaHub_DownloadStallTimeoutMs) {
				transferError = true;
				break;
			}
			const int64_t waitStartUs = esp_timer_get_time();
			delay(1);
			diagnostics.noDataWaitTimeUs += esp_timer_get_time() - waitStartUs;
			continue;
		}
		// wait for the writer task to finish draining this buffer before reusing it
		while (MediaHub_DownloadBufferFull[writeIndex]) {
			const int64_t waitStartUs = esp_timer_get_time();
			if (MediaHub_DownloadWriterError) {
				transferError = true;
				break;
			}
			if (System_IsRestartOrSleepPending()) {
				restartRequested = true;
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(1));
			diagnostics.writerWaitTimeUs += esp_timer_get_time() - waitStartUs;
		}
		if (transferError || restartRequested) {
			break;
		}

		const size_t bufferedSoFar = MediaHub_DownloadBufferBytes[writeIndex];
		const size_t spaceLeft = MediaHub_DownloadBufferSize - bufferedSoFar;
		const size_t toRead = std::min((size_t) avail, spaceLeft);
		const int64_t readStartUs = esp_timer_get_time();
		const size_t got = stream->readBytes(MediaHub_DownloadBuffers[writeIndex] + bufferedSoFar, toRead);
		diagnostics.httpReadTimeUs += esp_timer_get_time() - readStartUs;
		if (got == 0) {
			transferError = true;
			break;
		}
		lastDataMs = millis();
		++diagnostics.httpReadCalls;
		diagnostics.httpBytes += got;
		diagnostics.httpMaxChunk = std::max(diagnostics.httpMaxChunk, (uint32_t) got);
		MediaHub_RecordDownloadChunk(diagnostics.httpChunkHistogram, got);
		const int64_t hashStartUs = esp_timer_get_time();
		mbedtls_sha256_update(&shaCtx, MediaHub_DownloadBuffers[writeIndex] + bufferedSoFar, got);
		diagnostics.hashTimeUs += esp_timer_get_time() - hashStartUs;
		MediaHub_DownloadBufferBytes[writeIndex] = bufferedSoFar + got;
		totalRead += got;
		completedBytes += got;
		if (totalBytes > 0) {
			Led_SetDownloadProgress(true, (uint8_t) std::min<uint64_t>(100, (completedBytes * 100) / totalBytes));
		}
		if (MediaHub_DownloadBufferBytes[writeIndex] == MediaHub_DownloadBufferSize) {
			MediaHub_DownloadBufferFull[writeIndex] = true;
			writeIndex = (writeIndex + 1) % MediaHub_DownloadNumBuffers;
		}
		esp_task_wdt_reset();
	}
	http.end();

	// hand off whatever's left in the current buffer, then tell the writer
	// task we're done (2 = abort, 1 = normal finish - drain and exit either way)
	if (MediaHub_DownloadBufferBytes[writeIndex] > 0 && !MediaHub_DownloadBufferFull[writeIndex]) {
		MediaHub_DownloadBufferFull[writeIndex] = true;
	}
	xTaskNotify(MediaHub_DownloadWriterTaskHandle, (transferError || restartRequested) ? 2u : 1u, eSetValueWithOverwrite);
	if (xSemaphoreTake(MediaHub_DownloadWriterDone, pdMS_TO_TICKS(30000)) != pdTRUE) {
		// writer task got stuck somehow - don't hang forever, just report failure
		transferError = true;
	}
	if (MediaHub_DownloadWriterError) {
		transferError = true;
	}

	// The writer task has now stopped touching the buffers either way (done
	// or aborted) - safe to release them before this file's next steps run,
	// so a following file's TLS handshake doesn't have to compete with them.
	MediaHub_FreeDownloadBuffers();

	uint8_t digest[32];
	const int64_t hashFinishStartUs = esp_timer_get_time();
	mbedtls_sha256_finish(&shaCtx, digest);
	diagnostics.hashTimeUs += esp_timer_get_time() - hashFinishStartUs;
	mbedtls_sha256_free(&shaCtx);

	if (restartRequested) {
		gFSystem.remove(tmpPath);
		Log_Println(mediaHubDownloadAbortedForRestart, LOGLEVEL_NOTICE);
		return false;
	}

	if (transferError || totalRead != expectedSize) {
		gFSystem.remove(tmpPath);
		Log_Printf(LOGLEVEL_ERROR, mediaHubDownloadTransferError, fileUrl.c_str(), (unsigned) totalRead, (unsigned) expectedSize);
		return false;
	}

	if (!MediaHub_Sha256Hex(digest).equalsIgnoreCase(expectedSha256Hex)) {
		gFSystem.remove(tmpPath);
		Log_Printf(LOGLEVEL_ERROR, mediaHubVerifyFailed, fileUrl.c_str());
		return false;
	}

	gFSystem.remove(finalPath); // clears a stale leftover from an earlier aborted sync, if any
	const int64_t renameStartUs = esp_timer_get_time();
	const bool renamed = gFSystem.rename(tmpPath, finalPath);
	diagnostics.renameTimeUs = esp_timer_get_time() - renameStartUs;
	if (!renamed) {
		gFSystem.remove(tmpPath);
		Log_Printf(LOGLEVEL_ERROR, mediaHubDownloadRenameError, fileUrl.c_str());
		return false;
	}

	const uint32_t elapsedMs = millis() - transferStartMs;
	const float rateKBs = elapsedMs > 0 ? (totalRead / 1024.0f) / (elapsedMs / 1000.0f) : 0.0f;
	Log_Printf(LOGLEVEL_NOTICE, mediaHubDownloadRate, fileUrl.c_str(), (unsigned) totalRead, rateKBs);
	MediaHub_LogDownloadDiagnostics(diagnostics, totalRead, esp_timer_get_time() - downloadStartUs);

	// Temporary diagnostic (kept for easy re-enabling, not deleted): confirms
	// the per-file alloc/free of the download buffers isn't fragmenting the
	// internal heap over a multi-file sync. Re-enable if fragmentation is
	// ever suspected again - was confirmed stable across a multi-file test.
	// Log_Printf(LOGLEVEL_NOTICE, "MediaHub: internal heap largestFreeBlock=%u after %s", (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), fileUrl.c_str());

	return true;
}

// Downloads every file in `files` that isn't already fully synced in mediaDir.
// Checks free SD space against the total size of missing files up front
// (concept §13) so a card is never left half-downloaded due to running out
// of space mid-way through.
static bool MediaHub_SyncMissingFiles(const String &filesBaseUrl, const String &mediaDir, JsonArrayConst files,
	JsonArrayConst cachedFiles = JsonArrayConst(), uint16_t *downloadCount = nullptr) {
	// Total-vs-missing split doubles as the progress-bar baseline (concept
	// §7.1): a card that already has some files from an earlier partial sync
	// starts the bar accordingly, instead of jumping back to 0.
	uint64_t totalBytes = 0;
	uint64_t missingBytes = 0;
	uint16_t validFiles = 0;
	uint16_t missingFiles = 0;
	for (JsonVariantConst f : files) {
		const uint32_t size = f["size"] | 0;
		totalBytes += size;
		if (!MediaHub_FileValidForManifest(mediaDir, cachedFiles, f)) {
			missingBytes += size;
			++missingFiles;
		} else {
			++validFiles;
		}
	}
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub sync: %u required, %u valid, %u downloads", (unsigned) files.size(),
		(unsigned) validFiles, (unsigned) missingFiles);
	if (downloadCount != nullptr) {
		*downloadCount = missingFiles;
	}
	if (missingBytes == 0) {
		return true; // already fully synced
	}
	if (SdCard_GetFreeSize() < missingBytes) {
		Log_Println(mediaHubSdFull, LOGLEVEL_ERROR);
		return false;
	}

	MediaHub_BusyGuard busyGuard;
	uint64_t completedBytes = totalBytes - missingBytes;
	if (totalBytes > 0) {
		Led_SetDownloadProgress(true, (uint8_t) ((completedBytes * 100) / totalBytes));
	}
	for (JsonVariantConst f : files) {
		if (MediaHub_FileValidForManifest(mediaDir, cachedFiles, f)) {
			continue;
		}
		const char *path = f["path"] | "";
		const uint32_t size = f["size"] | 0;
		const char *sha256 = f["sha256"] | "";
		if (strlen(path) == 0 || strlen(sha256) == 0) {
			Log_Println(mediaHubInvalidManifest, LOGLEVEL_ERROR);
			return false;
		}
		Log_Printf(LOGLEVEL_NOTICE, mediaHubDownloadingFile, path);
		// URL-encode only for the HTTP request; the local path (mediaDir + path)
		// stays as-is for SanitizedFS, which does its own FAT-safe encoding.
		if (!MediaHub_DownloadAndVerifyFile(filesBaseUrl + MediaHub_UrlEncodePath(path), mediaDir + "/" + path, size, sha256, completedBytes, totalBytes)) {
			return false;
		}
	}
	return true;
}

// Writes the just-fetched manifest body to the local cache, best-effort: a
// failure here doesn't block playback since the manifest is already parsed
// in memory, it just means this device won't have an offline copy for the
// next tap.
static void MediaHub_WriteManifestCache(const char *cardId, const String &body, const String &hostPort = "") {
	File f = gFSystem.open(MediaHub_ManifestCachePath(cardId, hostPort), FILE_WRITE, true);
	if (!f) {
		return;
	}
	// The hub URL/key are local cache metadata, not a server-side contract.
	// They make a cached manifest independently syncable after a reboot and
	// allow legacy hidden storage to be migrated without an index database.
	JsonDocument doc;
	if (!deserializeJson(doc, body) && hostPort.length() > 0) {
		doc["mediaHubHubUrl"] = MediaHub_BuildBaseUrl(hostPort);
		doc["mediaHubHubKey"] = MediaHub_HubKey(hostPort);
		serializeJson(doc, f);
	} else {
		f.print(body);
	}
	f.close();
}

// AutoSync has just made these files locally usable. Persist the same compact
// RFID assignment format used by the normal web/RFID flow, so an unavailable
// hub on a later tap falls through to AudioPlayer without a second registry.
static bool MediaHub_PersistLocalAssignment(const char *cardId, const String &itemToPlay, uint32_t playMode,
	const TimeReleaseConfig *timeRelease = nullptr) {
	if (cardId == nullptr || itemToPlay.length() == 0) {
		return false;
	}
	// A refresh may change the path/playmode but must not throw away an
	// audiobook's persisted position just because it was maintained.
	String lastPlayPos = "0";
	String trackLastPlayed = "0";
	const String previous = gPrefsRfid.getString(cardId, "");
	const int first = previous.indexOf(stringDelimiter);
	const int second = previous.indexOf(stringDelimiter, first + 1);
	const int third = previous.indexOf(stringDelimiter, second + 1);
	const int fourth = previous.indexOf(stringDelimiter, third + 1);
	if (first == 0 && second > first && third > second && fourth > third) {
		lastPlayPos = previous.substring(second + 1, third);
		const int fifth = previous.indexOf(stringDelimiter, fourth + 1);
		trackLastPlayed = fifth > fourth ? previous.substring(fourth + 1, fifth) : previous.substring(fourth + 1);
	}
	if (playMode == TIME_RELEASE && (timeRelease == nullptr || !TimeRelease_IsConfigValid(*timeRelease))) {
		return false;
	}
	String assignment = String(stringDelimiter) + itemToPlay + stringDelimiter + lastPlayPos + stringDelimiter
		+ String(playMode) + stringDelimiter + trackLastPlayed;
	if (playMode == TIME_RELEASE) {
		assignment += String(stringDelimiter) + String(timeRelease->startTime) + stringDelimiter + String(timeRelease->intervalSecs);
		if (timeRelease->intervalUnit == TimeReleaseIntervalUnit::Months) {
			assignment += String(stringDelimiter) + String(timeRelease->intervalValue) + stringDelimiter + "months";
		}
	}
	if (gPrefsRfid.putString(cardId, assignment) == 0 || gPrefsRfid.getString(cardId, "") != assignment) {
		Log_Println("MediaHub: could not persist local assignment", LOGLEVEL_ERROR);
		return false;
	}
	Log_Println("MediaHub: local assignment saved", LOGLEVEL_NOTICE);
	return true;
}

static bool MediaHub_PersistAutoSyncFallback(const char *cardId, const String &mediaDir, JsonArrayConst files,
	uint32_t playMode, const TimeReleaseConfig *timeRelease) {
	return MediaHub_PersistLocalAssignment(cardId, MediaHub_BuildItemToPlay(mediaDir, playMode, files), playMode, timeRelease);
}

// Fetches missing files and then plays (concept §10 "EnsureCard" core, sans
// the "stale"/re-sync wipe which is Phase 6). Called after a live manifest
// fetch for a file-based (non-webradio) manifest.
static bool MediaHub_SyncAndPlay(const char *cardId, JsonDocument &doc, uint32_t manifestPlayMode, uint32_t lastPlayPos,
	uint16_t trackLastPlayed, const String &hostPort = "", bool requirePresentRfidForPlayback = false,
	bool persistAutoSyncFallback = false) {
	const char *filesBaseUrl = doc["filesBaseUrl"] | "";
	JsonArrayConst files = doc["files"].as<JsonArrayConst>();
	TimeReleaseConfig timeRelease;
	if (!MediaHub_GetTimeReleaseConfig(doc.as<JsonVariantConst>(), manifestPlayMode, timeRelease)) {
		return false;
	}
	const TimeReleaseConfig *timeReleasePtr = manifestPlayMode == TIME_RELEASE ? &timeRelease : nullptr;
	if (strlen(filesBaseUrl) == 0 || files.size() == 0) {
		Log_Println(mediaHubInvalidManifest, LOGLEVEL_ERROR);
		return false;
	}

	const String mediaDir = MediaHub_MediaDir(cardId, hostPort);
	// Keep the last committed manifest available while evaluating a newer one.
	// A changed playlist version or play mode must not invalidate unchanged
	// media; only path/size/SHA-256 are file identity.
	JsonDocument cachedManifest;
	JsonArrayConst cachedFiles;
	File cachedFile = gFSystem.open(MediaHub_ManifestCachePath(cardId, hostPort));
	if (cachedFile && !cachedFile.isDirectory() && !deserializeJson(cachedManifest, cachedFile)) {
		cachedFiles = cachedManifest["files"].as<JsonArrayConst>();
	}
	cachedFile.close();
	if (!MediaHub_SyncMissingFiles(filesBaseUrl, mediaDir, files, cachedFiles)) {
		return false;
	}
	if (persistAutoSyncFallback && !MediaHub_PersistAutoSyncFallback(cardId, mediaDir, files, manifestPlayMode, timeReleasePtr)) {
		return false;
	}

	const String itemToPlay = MediaHub_BuildItemToPlay(mediaDir, manifestPlayMode, files);
	// A sync deliberately continues after removal.  Only its final playback
	// decision shares the reader's existing pause-on-removal policy.
	if (requirePresentRfidForPlayback && gPlayProperties.pauseIfRfidRemoved && !Rfid_IsCardPresent()) {
		Log_Println("MediaHub AutoSync: playback skipped because RFID is no longer present", LOGLEVEL_NOTICE);
		Rfid_ResetLastTag(); // a later re-apply must dispatch the freshly synced manifest, not toggle an older playlist
		return true;
	}
	Log_Println(mediaHubPlayingAfterSync, LOGLEVEL_NOTICE);
	if (!AudioPlayer_SetPlaylist(itemToPlay.c_str(), lastPlayPos, manifestPlayMode, trackLastPlayed, timeReleasePtr)) {
		return false;
	}
	Rfid_MarkTagActivatedPlayback(cardId);
	if (requirePresentRfidForPlayback) {
		Log_Println("MediaHub AutoSync: playback started", LOGLEVEL_NOTICE);
	}
	return true;
}

// Re-sync flow for a card already marked "stale" (concept §11/§13): fetches
// the fresh manifest live, then wipes the media folder and re-downloads
// everything. Deliberately never starts playback on success (unlike the
// concept's original "RS -> PLAY" flowchart, §11): a re-sync can take
// minutes, and starting playback of whatever just finished downloading,
// unprompted, after that long a silent wait surprised more than it helped in
// practice - the next tap plays it instead, by then straight from the now-
// fully-synced local cache. Returns false for anything that leaves the OLD,
// still-complete local copy as the better fallback (hub unreachable, bad
// manifest, SD too full for the new version) — the caller then plays that
// old copy instead, and the card stays marked "stale" for the next attempt.
// Only clears "stale" on full success.
static bool MediaHub_TryReSync(const char *cardId, const String &hostPort) {
	String espId = MediaHub_GetEspId();
	String url = MediaHub_BuildBaseUrl(hostPort) + "/" + espId + "/card/" + String(cardId) + "/manifest.json";

	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_ReadTimeoutMs);
	if (!http.begin(url)) {
		return false;
	}
	const int httpCode = http.GET();
	if (httpCode != HTTP_CODE_OK) {
		http.end();
		return false;
	}
	const String body = http.getString();
	http.end();

	JsonDocument doc;
	if (deserializeJson(doc, body)) {
		return false;
	}
	const char *manifestCardId = doc["cardId"] | "";
	if (strcmp(manifestCardId, cardId) != 0) {
		return false;
	}

	Log_Println(mediaHubResyncing, LOGLEVEL_NOTICE);

	const uint32_t manifestPlayMode = doc["playMode"] | 0;
	if (manifestPlayMode == WEBSTREAM) {
		const char *stream = doc["stream"] | "";
		if (strlen(stream) == 0) {
			return false;
		}
		MediaHub_WriteManifestCache(cardId, body, hostPort);
		MediaHub_ClearStale(cardId, hostPort);
		Log_Println(mediaHubResyncComplete, LOGLEVEL_NOTICE);
		System_IndicateOk();
		return true;
	}

	const char *filesBaseUrl = doc["filesBaseUrl"] | "";
	JsonArrayConst files = doc["files"].as<JsonArrayConst>();
	if (strlen(filesBaseUrl) == 0 || files.size() == 0) {
		return false;
	}

	uint64_t totalNeeded = 0;
	for (JsonVariantConst f : files) {
		totalNeeded += (uint32_t) (f["size"] | 0);
	}

	const String mediaDir = MediaHub_MediaDir(cardId, hostPort);
	const uint64_t oldFolderSize = MediaHub_DirSize(mediaDir);
	if (SdCard_GetFreeSize() + oldFolderSize < totalNeeded) {
		// Old, working version stays untouched (nothing wiped yet); card
		// remains "stale" so this is retried on the next tap.
		Log_Println(mediaHubSdFull, LOGLEVEL_ERROR);
		return false;
	}

	File oldDir = gFSystem.open(mediaDir);
	if (oldDir && oldDir.isDirectory()) {
		MediaHub_DeleteDirRecursive(oldDir);
	}
	// Past this point there's no complete old copy left to fall back to: any
	// further failure leaves the card marked "stale" ("needs resync") for a
	// retry on the next tap, exactly like a fresh sync failure would.

	MediaHub_WriteManifestCache(cardId, body, hostPort);
	if (!MediaHub_SyncMissingFiles(filesBaseUrl, mediaDir, files)) {
		return false;
	}

	MediaHub_ClearStale(cardId, hostPort);
	Log_Println(mediaHubResyncComplete, LOGLEVEL_NOTICE);
	System_IndicateOk();
	return true;
}

struct MediaHub_VersionCheckArgs {
	String cardId;
	String hostPort;
};

// Runs on its own short-lived task so it never delays returning from
// MediaHub_HandleCardTapped (concept §9/§11: "Hintergrund"-check must not
// block the tap that's already playing). Only ever compares/marks — it
// never downloads or touches the manifest cache; the actual update happens
// via MediaHub_TryReSync() on a later tap.
static void MediaHub_VersionCheckTask(void *pvParameters) {
	auto *args = static_cast<MediaHub_VersionCheckArgs *>(pvParameters);

	String cachedVersion;
	File cacheFile = gFSystem.open(MediaHub_ManifestCachePath(args->cardId.c_str(), args->hostPort));
	if (cacheFile && !cacheFile.isDirectory()) {
		JsonDocument cachedDoc;
		if (!deserializeJson(cachedDoc, cacheFile)) {
			cachedVersion = String((const char *) (cachedDoc["version"] | ""));
		}
		cacheFile.close();
	}

	if (cachedVersion.length() > 0) {
		String espId = MediaHub_GetEspId();
		String url = MediaHub_BuildBaseUrl(args->hostPort) + "/" + espId + "/card/" + args->cardId + "/manifest.json";

		HTTPClient http;
		http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
		http.setTimeout(MediaHub_ReadTimeoutMs);
		if (http.begin(url)) {
			const int httpCode = http.GET();
			if (httpCode == HTTP_CODE_OK) {
				const String body = http.getString();
				JsonDocument doc;
				if (!deserializeJson(doc, body)) {
					const char *manifestCardId = doc["cardId"] | "";
					const char *freshVersion = doc["version"] | "";
					if (strcmp(manifestCardId, args->cardId.c_str()) == 0 && strlen(freshVersion) > 0 && cachedVersion != freshVersion) {
						MediaHub_MarkStale(args->cardId.c_str(), args->hostPort);
						Log_Println(mediaHubMarkedStale, LOGLEVEL_NOTICE);
					}
				}
			}
			http.end();
		}
	}

	delete args;
	vTaskDelete(NULL);
}

// Spawns MediaHub_VersionCheckTask() and returns immediately.
static void MediaHub_StartBackgroundVersionCheck(const char *cardId, const String &hostPort) {
	auto *args = new MediaHub_VersionCheckArgs {String(cardId), hostPort};
	xTaskCreatePinnedToCore(MediaHub_VersionCheckTask, "MediaHubVerChk", 4096, args, 1, NULL, 1);
}

// Local-first fast path (concept §3 principle 1): if a manifest is already
// cached and every one of its files is present with the right size, play
// immediately — no network access at all, so this works fully offline
// (e.g. in the car). Returns false if there's no cache yet, it doesn't
// parse, it's a webradio manifest (never offline-playable, §7.2/§14), or
// anything is missing/mismatched — the caller then falls back to asking
// the hub live.
static bool MediaHub_TryPlayFromLocalCache(const char *cardId, uint32_t lastPlayPos, uint16_t trackLastPlayed,
	const String &hostPort = "") {
	File manifestFile = gFSystem.open(MediaHub_ManifestCachePath(cardId, hostPort));
	if (!manifestFile || manifestFile.isDirectory()) {
		return false;
	}

	JsonDocument doc;
	const DeserializationError jsonError = deserializeJson(doc, manifestFile);
	manifestFile.close();
	if (jsonError) {
		Log_Printf(LOGLEVEL_ERROR, jsonErrorMsg, jsonError.c_str());
		return false; // corrupt cache -> treat as if there was none
	}

	const uint32_t manifestPlayMode = doc["playMode"] | 0;
	if (manifestPlayMode == WEBSTREAM) {
		return false;
	}

	const String mediaDir = MediaHub_MediaDir(cardId, hostPort);
	JsonArrayConst files = doc["files"].as<JsonArrayConst>();
	TimeReleaseConfig timeRelease;
	if (!MediaHub_GetTimeReleaseConfig(doc.as<JsonVariantConst>(), manifestPlayMode, timeRelease)) {
		return false;
	}
	const TimeReleaseConfig *timeReleasePtr = manifestPlayMode == TIME_RELEASE ? &timeRelease : nullptr;
	if (!MediaHub_AllFilesSynced(mediaDir, files)) {
		return false;
	}

	const String itemToPlay = MediaHub_BuildItemToPlay(mediaDir, manifestPlayMode, files);

	Log_Println(mediaHubPlayingFromCache, LOGLEVEL_NOTICE);
	if (!AudioPlayer_SetPlaylist(itemToPlay.c_str(), lastPlayPos, manifestPlayMode, trackLastPlayed, timeReleasePtr)) {
		return false;
	}
	Rfid_MarkTagActivatedPlayback(cardId);
	return true;
}

bool MediaHub_IsMediaHubPath(const char *path) {
	return path != NULL && strncmp(path, MediaHub_PathPrefix, strlen(MediaHub_PathPrefix)) == 0;
}

String MediaHub_GetEspId() {
	String mac = Wlan_GetMacAddress(); // "AA:BB:CC:DD:EE:FF" or empty if not available yet
	mac.replace(":", "");
	mac.toUpperCase();
	return mac;
}

// Dispatch entry point - see MediaHub.h for the contract. Order: local-cache
// fast path, then stale re-sync, then a live manifest fetch.
void MediaHub_HandleCardTapped(const char *cardId, const char *path, uint32_t lastPlayPos, uint16_t trackLastPlayed) {
	if (MediaHub_DownloadBusy.load(std::memory_order_relaxed)) {
		Log_Println(mediaHubBusy, LOGLEVEL_NOTICE);
		System_IndicateError();
		return;
	}

	if (!MediaHub_IsMediaHubPath(path)) {
		Log_Println(mediaHubInvalidPath, LOGLEVEL_ERROR);
		System_IndicateError();
		return;
	}

	String hostPort = String(path).substring(strlen(MediaHub_PathPrefix));
	if (hostPort.length() == 0) {
		Log_Println(mediaHubInvalidPath, LOGLEVEL_ERROR);
		System_IndicateError();
		return;
	}

	const bool online = Wlan_IsConnected();

	if (MediaHub_IsStale(cardId, hostPort) && online) {
		if (MediaHub_TryReSync(cardId, hostPort)) {
			return; // re-synced; this tap doesn't play, the next one does (see MediaHub_TryReSync())
		}
		// Re-sync wasn't possible right now (hub unreachable, bad manifest, SD
		// full for the new version, ...) — fall through and play the old,
		// still-complete local copy instead (concept §14: never block on a
		// transient hub failure while a working copy exists).
	}

	if (MediaHub_TryPlayFromLocalCache(cardId, lastPlayPos, trackLastPlayed, hostPort)) {
		if (online) {
			MediaHub_StartBackgroundVersionCheck(cardId, hostPort);
		}
		return;
	}

	if (!online) {
		Log_Println(mediaHubNotReachable, LOGLEVEL_ERROR);
		System_IndicateError();
		return;
	}

	String espId = MediaHub_GetEspId();
	String url = MediaHub_BuildBaseUrl(hostPort) + "/" + espId + "/card/" + String(cardId) + "/manifest.json";

	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_ReadTimeoutMs);
	if (!http.begin(url)) {
		Log_Println(mediaHubNotReachable, LOGLEVEL_ERROR);
		System_IndicateError();
		return;
	}

	const int httpCode = http.GET();
	if (httpCode <= 0) {
		// Transport-level failure (no route, connection refused, timeout, ...):
		// never block, just report and bail (concept §14).
		Log_Println(mediaHubNotReachable, LOGLEVEL_ERROR);
		System_IndicateError();
		http.end();
		return;
	}

	const String body = http.getString();
	http.end();

	if (httpCode == 404) {
		// Unknown / not-yet-assigned card: the hub has (re-)registered it as
		// pending for the admin to assign (concept §5.3). Not an error the
		// user can fix here, just make it visible.
		Log_Println(mediaHubCardPending, LOGLEVEL_NOTICE);
		System_IndicateError();
		return;
	}

	if (httpCode != 200) {
		Log_Printf(LOGLEVEL_ERROR, mediaHubUnexpectedStatus, httpCode);
		System_IndicateError();
		return;
	}

	JsonDocument doc;
	const DeserializationError jsonError = deserializeJson(doc, body);
	if (jsonError) {
		Log_Printf(LOGLEVEL_ERROR, jsonErrorMsg, jsonError.c_str());
		System_IndicateError();
		return;
	}

	// Cross-check that the manifest actually belongs to the tapped card
	// (concept §7.1) before trusting/caching any of it.
	const char *manifestCardId = doc["cardId"] | "";
	if (strcmp(manifestCardId, cardId) != 0) {
		Log_Println(mediaHubInvalidManifest, LOGLEVEL_ERROR);
		System_IndicateError();
		return;
	}
	const uint32_t manifestPlayMode = doc["playMode"] | 0;
	if (manifestPlayMode == WEBSTREAM) {
		const char *stream = doc["stream"] | "";
		if (strlen(stream) == 0) {
			Log_Println(mediaHubInvalidManifest, LOGLEVEL_ERROR);
			System_IndicateError();
			return;
		}
		MediaHub_WriteManifestCache(cardId, body, hostPort);
		Log_Println(mediaHubWebstreamFromManifest, LOGLEVEL_NOTICE);
		// Webradio manifests have no files to sync and no play-position to
		// track (concept §7.2) — hand the stream straight to the existing
		// webradio playback path, same as a native WEBSTREAM card.
		if (AudioPlayer_SetPlaylist(stream, 0, WEBSTREAM, 0)) {
			Rfid_MarkTagActivatedPlayback(cardId);
		}
		MediaHub_StartBackgroundVersionCheck(cardId, hostPort);
		return;
	}

	if (MediaHub_SyncAndPlay(cardId, doc, manifestPlayMode, lastPlayPos, trackLastPlayed, hostPort)) {
		MediaHub_WriteManifestCache(cardId, body, hostPort);
		MediaHub_StartBackgroundVersionCheck(cardId, hostPort);
	} else {
		System_IndicateError();
	}
}

// Removes everything MediaHub keeps locally for one card: manifest cache,
// stale marker, and downloaded media. Shared by MediaHub_ForceRefresh() (the
// card comes back on the next tap) and MediaHub_DeleteCard() (it doesn't).
static bool MediaHub_WipeCard(const char *cardId) {
	gFSystem.remove(MediaHub_ManifestCachePath(cardId));
	MediaHub_ClearStale(cardId);
	File mediaDir = gFSystem.open(MediaHub_MediaDir(cardId));
	if (!mediaDir || !mediaDir.isDirectory()) {
		return true; // nothing to wipe
	}
	return MediaHub_DeleteDirRecursive(mediaDir);
}

// Escape-hatch (concept §9/#7): discards the local cache for one card so the
// next tap re-fetches the manifest and re-downloads everything from scratch.
// Not wired to a trigger yet (Admin-Karte / Hub-Button lands in a later
// phase alongside the "stale" mechanism it shares its machinery with).
bool MediaHub_ForceRefresh(const char *cardId) {
	return MediaHub_WipeCard(cardId);
}

// REST-cascade target for DELETE /rfid?id=<cardId> (concept §13.1): called by
// Web.cpp's handleDeleteRFIDRequest() for MediaHub-managed cards, in addition
// to (not instead of) removing the NVS entry itself.
bool MediaHub_DeleteCard(const char *cardId) {
	return MediaHub_WipeCard(cardId);
}

// Same escape-hatch for every MediaHub-managed card at once ("für alle",
// concept §9).
bool MediaHub_ForceRefreshAll() {
	bool ok = true;
	File manifestsDir = gFSystem.open("/.mediahub/manifests");
	if (manifestsDir && manifestsDir.isDirectory()) {
		ok &= MediaHub_DeleteDirRecursive(manifestsDir);
	}
	File mediaRootDir = gFSystem.open("/.mediahub/media");
	if (mediaRootDir && mediaRootDir.isDirectory()) {
		ok &= MediaHub_DeleteDirRecursive(mediaRootDir);
	}
	return ok;
}

// Registered media servers (concept §5.1): stored as one JSON array under a
// single settings key rather than mirroring Wlan.cpp's per-entry NVS-key
// scheme, since this is always a short list of two-string records - a
// dedicated per-entry NVS layout would be complexity this doesn't need.
static constexpr const char *MediaHub_ServersNvsKey = "mediaHubSrvs";
static constexpr uint8_t MediaHub_MaxServers = 10;

bool MediaHub_IsAutoSyncEnabled() {
	return gPrefsSettings.getBool(MediaHub_AutoSyncNvsKey, false);
}

bool MediaHub_SetAutoSyncEnabled(bool enabled, size_t *written) {
	const size_t result = gPrefsSettings.putBool(MediaHub_AutoSyncNvsKey, enabled);
	if (written != nullptr) {
		*written = result;
	}
	return result == sizeof(bool) && gPrefsSettings.getBool(MediaHub_AutoSyncNvsKey, !enabled) == enabled;
}

bool MediaHub_IsVisibleStorageEnabled() {
	return gPrefsSettings.getBool(MediaHub_VisibleStorageNvsKey, false);
}

bool MediaHub_SetVisibleStorageEnabled(bool enabled, size_t *written) {
	const size_t result = gPrefsSettings.putBool(MediaHub_VisibleStorageNvsKey, enabled);
	if (written != nullptr) {
		*written = result;
	}
	return result == sizeof(bool) && gPrefsSettings.getBool(MediaHub_VisibleStorageNvsKey, !enabled) == enabled;
}

bool MediaHub_UseVisibleStorage() {
	return MediaHub_IsAutoSyncEnabled() || MediaHub_IsVisibleStorageEnabled();
}

std::vector<MediaHubServer> MediaHub_GetServers() {
	std::vector<MediaHubServer> servers;
	const String json = gPrefsSettings.getString(MediaHub_ServersNvsKey, "[]");
	JsonDocument doc;
	if (deserializeJson(doc, json)) {
		return servers; // corrupt/missing -> treat as empty
	}
	for (JsonVariantConst entry : doc.as<JsonArrayConst>()) {
		MediaHubServer s;
		s.name = entry["name"] | "";
		s.hostPort = entry["hostPort"] | "";
		s.https = entry["https"] | false; // absent (servers registered before https support) -> http
		s.alias = entry["alias"] | ""; // absent in existing configurations -> technical hub-key fallback
		s.alias.trim();
		if (s.name.length() > 0 && s.hostPort.length() > 0) {
			servers.push_back(s);
		}
	}
	return servers;
}

// Writes the full list back as one JSON blob (see MediaHub_ServersNvsKey).
static bool MediaHub_SaveServers(const std::vector<MediaHubServer> &servers) {
	JsonDocument doc;
	JsonArray arr = doc.to<JsonArray>();
	for (const auto &s : servers) {
		JsonObject o = arr.add<JsonObject>();
		o["name"] = s.name;
		o["hostPort"] = s.hostPort;
		o["https"] = s.https;
		o["alias"] = s.alias;
	}
	String json;
	serializeJson(doc, json);
	return gPrefsSettings.putString(MediaHub_ServersNvsKey, json) == json.length();
}

static bool MediaHub_IsValidAlias(const String &alias) {
	if (alias.length() > 64) {
		return false;
	}
	for (size_t index = 0; index < alias.length(); ++index) {
		const char c = alias[index];
		if (!(isalnum(static_cast<unsigned char>(c)) || c == ' ' || c == '-' || c == '_')) {
			return false;
		}
	}
	return true;
}

bool MediaHub_SaveServer(const String &name, const String &hostPort, bool https, const String &alias) {
	if (name.length() == 0 || hostPort.length() == 0) {
		return false;
	}
	String normalizedAlias = alias;
	normalizedAlias.trim();
	if (!MediaHub_IsValidAlias(normalizedAlias)) {
		return false;
	}
	std::vector<MediaHubServer> servers = MediaHub_GetServers();
	for (const MediaHubServer &s : servers) {
		if (s.name != name && normalizedAlias.length() > 0 && s.alias.equalsIgnoreCase(normalizedAlias)) {
			return false; // avoid a visible-folder collision; never synthesize suffixes
		}
	}
	for (auto &s : servers) {
		if (s.name == name) {
			s.hostPort = hostPort;
			s.https = https;
			s.alias = normalizedAlias;
			return MediaHub_SaveServers(servers);
		}
	}
	if (servers.size() >= MediaHub_MaxServers) {
		return false;
	}
	servers.push_back({name, hostPort, https, normalizedAlias});
	return MediaHub_SaveServers(servers);
}

bool MediaHub_DeleteServer(const String &name) {
	std::vector<MediaHubServer> servers = MediaHub_GetServers();
	const size_t before = servers.size();
	servers.erase(std::remove_if(servers.begin(), servers.end(), [&name](const MediaHubServer &s) { return s.name == name; }), servers.end());
	if (servers.size() == before) {
		return false; // nothing to delete
	}
	return MediaHub_SaveServers(servers);
}

enum class MediaHub_ResolveResult : uint8_t {
	Unavailable,
	NotAssigned,
	Assigned,
};

// Probe endpoints are deliberately separate from manifest.json: searching a
// lower-priority hub must never create a pending card there.
static MediaHub_ResolveResult MediaHub_ResolveCard(const String &hostPort, const char *cardId) {
	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_ReadTimeoutMs);
	const String url = MediaHub_BuildBaseUrl(hostPort) + "/" + MediaHub_GetEspId() + "/card/" + String(cardId) + "/resolve";
	if (!http.begin(url)) {
		return MediaHub_ResolveResult::Unavailable;
	}
	const int status = http.GET();
	const String body = status > 0 ? http.getString() : "";
	http.end();
	if (status != HTTP_CODE_OK) {
		return status == HTTP_CODE_NOT_FOUND ? MediaHub_ResolveResult::NotAssigned : MediaHub_ResolveResult::Unavailable;
	}
	JsonDocument doc;
	if (deserializeJson(doc, body) || !doc["assigned"].is<bool>()) {
		return MediaHub_ResolveResult::Unavailable;
	}
	return doc["assigned"].as<bool>() ? MediaHub_ResolveResult::Assigned : MediaHub_ResolveResult::NotAssigned;
}

static void MediaHub_SendCardSeen(const String &hostPort, const char *cardId) {
	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_ReadTimeoutMs);
	const String url = MediaHub_BuildBaseUrl(hostPort) + "/" + MediaHub_GetEspId() + "/card/" + String(cardId) + "/seen";
	if (http.begin(url)) {
		http.addHeader("Content-Type", "application/json");
		http.POST("{}");
		http.end();
	}
}

static bool MediaHub_SyncAutoCard(const char *cardId, const String &hostPort) {
	if (MediaHub_DownloadBusy.load(std::memory_order_relaxed)) {
		Log_Println(mediaHubBusy, LOGLEVEL_NOTICE);
		System_IndicateError();
		return false;
	}
	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_ReadTimeoutMs);
	const String url = MediaHub_BuildBaseUrl(hostPort) + "/" + MediaHub_GetEspId() + "/card/" + String(cardId) + "/manifest.json";
	if (!http.begin(url)) {
		Log_Println(mediaHubNotReachable, LOGLEVEL_ERROR);
		System_IndicateError();
		return false;
	}
	const int status = http.GET();
	const String body = status > 0 ? http.getString() : "";
	http.end();
	if (status != HTTP_CODE_OK) {
		Log_Printf(LOGLEVEL_ERROR, mediaHubUnexpectedStatus, status);
		System_IndicateError();
		return false;
	}
	JsonDocument doc;
	if (deserializeJson(doc, body) || strcmp(doc["cardId"] | "", cardId) != 0) {
		Log_Println(mediaHubInvalidManifest, LOGLEVEL_ERROR);
		System_IndicateError();
		return false;
	}
	const uint32_t playMode = doc["playMode"] | 0;
	if (playMode == WEBSTREAM) {
		const char *stream = doc["stream"] | "";
		if (strlen(stream) == 0) {
			Log_Println(mediaHubInvalidManifest, LOGLEVEL_ERROR);
			System_IndicateError();
			return false;
		}
		MediaHub_WriteManifestCache(cardId, body, hostPort);
		if (!MediaHub_PersistLocalAssignment(cardId, stream, WEBSTREAM)) {
			System_IndicateError();
			return false;
		}
		if (gPlayProperties.pauseIfRfidRemoved && !Rfid_IsCardPresent()) {
			Log_Println("MediaHub AutoSync: playback skipped because RFID is no longer present", LOGLEVEL_NOTICE);
			Rfid_ResetLastTag();
			return true;
		}
		if (!AudioPlayer_SetPlaylist(stream, 0, WEBSTREAM, 0)) {
			return false;
		}
		Rfid_MarkTagActivatedPlayback(cardId);
		Log_Println("MediaHub AutoSync: playback started", LOGLEVEL_NOTICE);
		return true;
	}

	// AutoSync reaches this point before RfidCommon may dispatch a local
	// playlist.  A required transfer therefore cannot briefly play stale
	// local content.
	if (!MediaHub_SyncAndPlay(cardId, doc, playMode, 0, 0, hostPort, true, true)) {
		System_IndicateError();
		return false;
	}
	MediaHub_WriteManifestCache(cardId, body, hostPort);
	Log_Println("MediaHub AutoSync: sync completed", LOGLEVEL_NOTICE);
	return true;
}

bool MediaHub_TryAutoSync(const char *cardId, const char *fallbackPath, uint32_t fallbackLastPlayPos,
	uint32_t fallbackPlayMode, uint16_t fallbackTrackLastPlayed) {
	(void) fallbackPath;
	(void) fallbackLastPlayPos;
	(void) fallbackPlayMode;
	(void) fallbackTrackLastPlayed;
	if (!MediaHub_IsAutoSyncEnabled() || !Wlan_IsConnected() || cardId == nullptr) {
		return false;
	}

	std::vector<String> reachableMisses;
	const std::vector<MediaHubServer> servers = MediaHub_GetServers();
	for (size_t index = 0; index < servers.size(); ++index) {
		const MediaHubServer &server = servers[index];
		const String hostPort = String(server.https ? "https://" : "http://") + server.hostPort;
		const MediaHub_ResolveResult result = MediaHub_ResolveCard(hostPort, cardId);
		if (result == MediaHub_ResolveResult::Assigned) {
			Log_Printf(LOGLEVEL_NOTICE, "MediaHub AutoSync: card=%s matched hub=%s rank=%u", cardId, server.name.c_str(), (unsigned) (index + 1));
			MediaHub_SyncAutoCard(cardId, hostPort);
			return true; // first assigned hub wins; no lower priority request follows
		}
		if (result == MediaHub_ResolveResult::NotAssigned) {
			reachableMisses.push_back(hostPort);
		}
	}
	for (const String &hostPort : reachableMisses) {
		MediaHub_SendCardSeen(hostPort, cardId);
	}
	return false;
}

// File::name() is only a basename on the SD implementation used here. Keep
// SanitizedFS's reparsed absolute File::path() while descending so flat and
// hub-keyed manifest trees both reach the sync operation unambiguously.
// There is deliberately no global index: the manifests are the durable source
// of truth for explicit maintenance synchronization.
static void MediaHub_CollectManifestPaths(File dir, std::vector<String> &paths) {
	File entry = dir.openNextFile();
	while (entry) {
		if (entry.isDirectory()) {
			MediaHub_CollectManifestPaths(entry, paths);
		} else {
			const String path = gFSystem.path(entry);
			if (path.endsWith(".json")) {
				paths.push_back(path);
			}
		}
		entry = dir.openNextFile();
		esp_task_wdt_reset();
	}
}

// Read active RFID assignment keys from the existing NVS namespace.  A
// cleanup must derive references from NVS, never from manifest filenames.
static bool MediaHub_CollectActiveRfidCards(std::vector<String> &cards) {
	constexpr const char *partition = "nvs";
	constexpr const char *rfidNamespace = "rfidTags";
#if (defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3))
	nvs_iterator_t it = nullptr;
	esp_err_t result = nvs_entry_find(partition, rfidNamespace, NVS_TYPE_ANY, &it);
	if (result != ESP_OK && result != ESP_ERR_NVS_NOT_FOUND) {
		return false;
	}
	while (result == ESP_OK) {
		nvs_entry_info_t info;
		nvs_entry_info(it, &info);
		if (isNumber(info.key) && gPrefsRfid.getString(info.key, "").length() > 0) {
			cards.push_back(info.key);
		}
		result = nvs_entry_next(&it);
	}
	nvs_release_iterator(it);
	return result == ESP_ERR_NVS_NOT_FOUND;
#else
	nvs_iterator_t it = nvs_entry_find(partition, rfidNamespace, NVS_TYPE_ANY);
	while (it != nullptr) {
		nvs_entry_info_t info;
		nvs_entry_info(it, &info);
		if (isNumber(info.key) && gPrefsRfid.getString(info.key, "").length() > 0) {
			cards.push_back(info.key);
		}
		it = nvs_entry_next(it);
	}
	return true;
#endif
}

static bool MediaHub_IsActiveRfidCard(const std::vector<String> &cards, const String &cardId) {
	return std::find(cards.begin(), cards.end(), cardId) != cards.end();
}

static bool MediaHub_ReadManifestCardId(const String &manifestPath, String &cardId) {
	File manifestFile = gFSystem.open(manifestPath);
	if (!manifestFile || manifestFile.isDirectory()) {
		return false;
	}
	JsonDocument manifest;
	const DeserializationError error = deserializeJson(manifest, manifestFile);
	manifestFile.close();
	if (error) {
		return false;
	}
	cardId = manifest["cardId"] | "";
	return cardId.length() > 0 && isNumber(cardId.c_str());
}

bool MediaHub_CleanupAllHidden() {
	if (MediaHub_DownloadBusy.load(std::memory_order_relaxed) || MediaHub_SyncBusy.load(std::memory_order_relaxed)) {
		Log_Println("MediaHub cleanup: refused while synchronization is active", LOGLEVEL_NOTICE);
		return false;
	}
	Log_Println("MediaHub cleanup: removing all hidden MediaHub cache data", LOGLEVEL_NOTICE);
	bool ok = MediaHub_DeleteHiddenDir("/.mediahub/manifests");
	ok &= MediaHub_DeleteHiddenDir("/.mediahub/media");
	if (ok) {
		Log_Println("MediaHub cleanup: completed", LOGLEVEL_NOTICE);
	}
	return ok;
}

bool MediaHub_CleanupOrphanedHidden() {
	if (MediaHub_DownloadBusy.load(std::memory_order_relaxed) || MediaHub_SyncBusy.load(std::memory_order_relaxed)) {
		Log_Println("MediaHub cleanup: refused while synchronization is active", LOGLEVEL_NOTICE);
		return false;
	}

	std::vector<String> activeCards;
	if (!MediaHub_CollectActiveRfidCards(activeCards)) {
		Log_Println("MediaHub cleanup: refused because RFID NVS could not be enumerated", LOGLEVEL_ERROR);
		return false;
	}
	std::vector<String> manifests;
	File root = gFSystem.open("/.mediahub/manifests");
	if (root && root.isDirectory()) {
		MediaHub_CollectManifestPaths(root, manifests);
	}

	// Analyze the complete cache before changing anything. Invalid or ambiguous
	// manifests are retained: conservative cleanup is more important than space.
	std::vector<String> orphanedManifests;
	std::vector<String> orphanedMediaDirs;
	for (const String &manifestPath : manifests) {
		String cardId;
		if (!MediaHub_ReadManifestCardId(manifestPath, cardId)) {
			Log_Printf(LOGLEVEL_NOTICE, "MediaHub cleanup: retaining ambiguous manifest: %s", manifestPath.c_str());
			continue;
		}
		if (MediaHub_IsActiveRfidCard(activeCards, cardId)) {
			continue;
		}
		orphanedManifests.push_back(manifestPath);
		const String mediaDir = "/.mediahub/media/" + cardId;
		if (gFSystem.exists(mediaDir)) {
			orphanedMediaDirs.push_back(mediaDir);
		}
	}
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub cleanup: %u hidden manifests, %u active, %u orphaned", (unsigned) manifests.size(),
		(unsigned) activeCards.size(), (unsigned) orphanedManifests.size());

	bool ok = true;
	uint16_t removedMediaDirs = 0;
	for (const String &manifestPath : orphanedManifests) {
		String cardId;
		MediaHub_ReadManifestCardId(manifestPath, cardId); // already validated during analysis
		ok &= MediaHub_RemoveHiddenFile(manifestPath);
		ok &= MediaHub_RemoveHiddenFile(manifestPath + ".stale");
		Log_Printf(LOGLEVEL_NOTICE, "MediaHub cleanup: removed card=%s", cardId.c_str());
	}
	for (const String &mediaDir : orphanedMediaDirs) {
		if (MediaHub_DeleteHiddenDir(mediaDir)) {
			++removedMediaDirs;
		} else {
			ok = false;
		}
	}
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub cleanup: completed: %u manifests, %u cache directories removed",
		(unsigned) orphanedManifests.size(), (unsigned) removedMediaDirs);
	return ok;
}

static void MediaHub_LogSyncFailed(const char *cardId, const String &reason) {
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub Sync: card=%s failed: %s", (cardId != nullptr && strlen(cardId) > 0) ? cardId : "?", reason.c_str());
}

static bool MediaHub_WriteCachedManifestAtPath(const String &path, JsonDocument &manifest) {
	File file = gFSystem.open(path, FILE_WRITE, true);
	if (!file) {
		return false;
	}
	const size_t written = serializeJson(manifest, file);
	file.close();
	return written > 0;
}

static bool MediaHub_WriteManifestCacheAtPath(const String &path, const String &body, const String &hostPort) {
	JsonDocument manifest;
	if (deserializeJson(manifest, body)) {
		return false;
	}
	manifest["mediaHubHubUrl"] = MediaHub_BuildBaseUrl(hostPort);
	manifest["mediaHubHubKey"] = MediaHub_HubKey(hostPort);
	return MediaHub_WriteCachedManifestAtPath(path, manifest);
}

static String MediaHub_HubKeyFromManifestPath(const String &manifestPath) {
	static constexpr char prefix[] = "/.mediahub/manifests/";
	if (!manifestPath.startsWith(prefix)) {
		return "";
	}
	const String remainder = manifestPath.substring(sizeof(prefix) - 1);
	const int slash = remainder.indexOf('/');
	return slash > 0 ? remainder.substring(0, slash) : "";
}

enum class MediaHub_ManifestHubResult : uint8_t {
	Found,
	MissingUrl,
	MissingKey,
	MissingUrlAndKey,
	NotConfigured,
};

// Prefer persisted metadata, then the visible-storage path. A flat legacy
// cache has neither; it is migrated only if exactly one hub is configured.
static MediaHub_ManifestHubResult MediaHub_FindManifestHub(JsonDocument &manifest, const String &manifestPath,
	String &hostPort, bool &metadataMigrated) {
	const String cachedUrl = manifest["mediaHubHubUrl"] | "";
	const String cachedKey = manifest["mediaHubHubKey"] | "";
	const String pathKey = MediaHub_HubKeyFromManifestPath(manifestPath);
	const String effectiveKey = cachedKey.length() > 0 ? cachedKey : pathKey;
	const std::vector<MediaHubServer> servers = MediaHub_GetServers();
	for (const MediaHubServer &server : servers) {
		const String candidate = String(server.https ? "https://" : "http://") + server.hostPort;
		const String candidateUrl = MediaHub_BuildBaseUrl(candidate);
		const String candidateKey = MediaHub_HubKey(candidate);
		if (cachedUrl == candidateUrl || effectiveKey == candidateKey) {
			hostPort = candidate;
			if (cachedUrl != candidateUrl || cachedKey != candidateKey) {
				manifest["mediaHubHubUrl"] = candidateUrl;
				manifest["mediaHubHubKey"] = candidateKey;
				metadataMigrated = true;
			}
			return MediaHub_ManifestHubResult::Found;
		}
	}
	if (cachedUrl.length() > 0 || effectiveKey.length() > 0) {
		return MediaHub_ManifestHubResult::NotConfigured;
	}
	if (servers.size() == 1) {
		hostPort = String(servers[0].https ? "https://" : "http://") + servers[0].hostPort;
		manifest["mediaHubHubUrl"] = MediaHub_BuildBaseUrl(hostPort);
		manifest["mediaHubHubKey"] = MediaHub_HubKey(hostPort);
		metadataMigrated = true;
		return MediaHub_ManifestHubResult::Found;
	}
	if (cachedUrl.length() == 0 && cachedKey.length() == 0) {
		return MediaHub_ManifestHubResult::MissingUrlAndKey;
	}
	return cachedUrl.length() == 0 ? MediaHub_ManifestHubResult::MissingUrl : MediaHub_ManifestHubResult::MissingKey;
}

// Synchronizes a cached manifest without using AudioPlayer. Existing playback
// continues untouched; the established foreground download/.part mechanism is
// reused because it is the only SD/network transfer path already safe here.
static bool MediaHub_SyncCachedManifest(const String &manifestPath, uint16_t &downloads, bool &changed) {
	File cachedFile = gFSystem.open(manifestPath);
	if (!cachedFile || cachedFile.isDirectory()) {
		MediaHub_LogSyncFailed(nullptr, "local manifest file is not readable: " + manifestPath);
		return false;
	}
	JsonDocument cached;
	const DeserializationError cachedError = deserializeJson(cached, cachedFile);
	cachedFile.close();
	if (cachedError) {
		MediaHub_LogSyncFailed(nullptr, "local manifest JSON is invalid: " + manifestPath);
		return false;
	}

	const char *cardId = cached["cardId"] | "";
	if (strlen(cardId) == 0) {
		MediaHub_LogSyncFailed(nullptr, "missing card ID");
		return false;
	}
	String hostPort;
	bool metadataMigrated = false;
	const MediaHub_ManifestHubResult hubResult = MediaHub_FindManifestHub(cached, manifestPath, hostPort, metadataMigrated);
	if (hubResult != MediaHub_ManifestHubResult::Found) {
		switch (hubResult) {
			case MediaHub_ManifestHubResult::MissingUrl:
				MediaHub_LogSyncFailed(cardId, "missing hub URL");
				break;
			case MediaHub_ManifestHubResult::MissingKey:
				MediaHub_LogSyncFailed(cardId, "missing hub key");
				break;
			case MediaHub_ManifestHubResult::MissingUrlAndKey:
				MediaHub_LogSyncFailed(cardId, "missing hub metadata (hub URL and hub key)");
				break;
			case MediaHub_ManifestHubResult::NotConfigured:
				MediaHub_LogSyncFailed(cardId, "hub metadata cannot be mapped to a configured MediaHub");
				break;
			default:
				MediaHub_LogSyncFailed(cardId, "unknown hub lookup error");
				break;
		}
		return false;
	}
	if (metadataMigrated) {
		if (!MediaHub_WriteCachedManifestAtPath(manifestPath, cached)) {
			MediaHub_LogSyncFailed(cardId, "could not persist migrated hub metadata");
			return false;
		}
		Log_Printf(LOGLEVEL_NOTICE, "MediaHub Sync: card=%s migrated missing hub metadata", cardId);
	}

	HTTPClient http;
	http.setConnectTimeout(MediaHub_ConnectTimeoutMs);
	http.setTimeout(MediaHub_ReadTimeoutMs);
	const String url = MediaHub_BuildBaseUrl(hostPort) + "/" + MediaHub_GetEspId() + "/card/" + String(cardId) + "/manifest.json";
	if (!http.begin(url)) {
		MediaHub_LogSyncFailed(cardId, "MediaHub is not reachable");
		return false;
	}
	const int status = http.GET();
	const String body = status > 0 ? http.getString() : "";
	http.end();
	if (status != HTTP_CODE_OK) {
		MediaHub_LogSyncFailed(cardId, "remote manifest request failed (HTTP " + String(status) + ")");
		return false;
	}

	JsonDocument remote;
	if (deserializeJson(remote, body) || strcmp(remote["cardId"] | "", cardId) != 0) {
		MediaHub_LogSyncFailed(cardId, "remote manifest is invalid");
		return false;
	}
	const uint32_t playMode = remote["playMode"] | 0;
	TimeReleaseConfig timeRelease;
	if (!MediaHub_GetTimeReleaseConfig(remote.as<JsonVariantConst>(), playMode, timeRelease)) {
		MediaHub_LogSyncFailed(cardId, "remote TIME_RELEASE timing is invalid");
		return false;
	}
	const TimeReleaseConfig *timeReleasePtr = playMode == TIME_RELEASE ? &timeRelease : nullptr;
	changed = String(cached["version"] | "") != String(remote["version"] | "")
		|| (uint32_t) (cached["playMode"] | 0) != playMode
		|| (playMode == TIME_RELEASE && ((uint32_t) (cached["timeReleaseStart"] | 0) != timeRelease.startTime || (uint32_t) (cached["timeReleaseInterval"] | 0) != timeRelease.intervalSecs || String(cached["timeReleaseIntervalUnit"] | "") != String(remote["timeReleaseIntervalUnit"] | "") || (uint32_t) (cached["timeReleaseIntervalValue"] | 0) != timeRelease.intervalValue));
	if (playMode == WEBSTREAM) {
		const char *stream = remote["stream"] | "";
		if (strlen(stream) == 0 || !MediaHub_PersistLocalAssignment(cardId, stream, WEBSTREAM)) {
			MediaHub_LogSyncFailed(cardId, "sync or local assignment update failed");
			return false;
		}
		if (!MediaHub_WriteManifestCacheAtPath(manifestPath, body, hostPort)) {
			MediaHub_LogSyncFailed(cardId, "could not update local manifest");
			return false;
		}
		return true;
	}

	const char *filesBaseUrl = remote["filesBaseUrl"] | "";
	JsonArrayConst files = remote["files"].as<JsonArrayConst>();
	if (strlen(filesBaseUrl) == 0 || files.size() == 0) {
		MediaHub_LogSyncFailed(cardId, "remote manifest is invalid or incomplete");
		return false;
	}
	const String mediaDir = MediaHub_MediaDir(cardId, hostPort);
	JsonArrayConst cachedFiles = cached["files"].as<JsonArrayConst>();
	if (!MediaHub_SyncMissingFiles(filesBaseUrl, mediaDir, files, cachedFiles, &downloads)) {
		MediaHub_LogSyncFailed(cardId, "sync or file transfer failed");
		return false;
	}
	if (!MediaHub_PersistLocalAssignment(cardId, MediaHub_BuildItemToPlay(mediaDir, playMode, files), playMode, timeReleasePtr)) {
		MediaHub_LogSyncFailed(cardId, "local RFID assignment update failed");
		return false;
	}
	if (!MediaHub_WriteManifestCacheAtPath(manifestPath, body, hostPort)) {
		MediaHub_LogSyncFailed(cardId, "could not update local manifest");
		return false;
	}
	changed = changed || downloads > 0;
	return true;
}

void MediaHub_SyncLocalManifests() {
	if (MediaHub_DownloadBusy.load(std::memory_order_relaxed) || MediaHub_SyncBusy.load(std::memory_order_relaxed)) {
		Log_Println(mediaHubBusy, LOGLEVEL_NOTICE);
		System_IndicateError();
		return;
	}
	if (!Wlan_IsConnected()) {
		Log_Println("MediaHub Sync: WiFi unavailable", LOGLEVEL_NOTICE);
		System_IndicateError();
		return;
	}
	MediaHub_SyncBusyGuard syncBusyGuard;

	std::vector<String> manifests;
	File root = gFSystem.open("/.mediahub/manifests");
	if (root && root.isDirectory()) {
		MediaHub_CollectManifestPaths(root, manifests);
	}
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub Sync: %u local manifests", (unsigned) manifests.size());
	uint16_t updated = 0;
	uint16_t unchanged = 0;
	uint16_t failed = 0;
	for (const String &manifestPath : manifests) {
		uint16_t downloads = 0;
		bool changed = false;
		if (!MediaHub_SyncCachedManifest(manifestPath, downloads, changed)) {
			++failed;
			continue;
		}
		if (changed) {
			++updated;
			Log_Printf(LOGLEVEL_NOTICE, "MediaHub Sync: %s %u downloads", manifestPath.c_str(), (unsigned) downloads);
		} else {
			++unchanged;
			Log_Printf(LOGLEVEL_NOTICE, "MediaHub Sync: %s up to date", manifestPath.c_str());
		}
	}
	Log_Printf(LOGLEVEL_NOTICE, "MediaHub Sync: completed: %u updated, %u unchanged, %u failed", (unsigned) updated,
		(unsigned) unchanged, (unsigned) failed);
	if (failed == 0) {
		System_IndicateOk();
	} else {
		System_IndicateError();
	}
}

bool MediaHub_MoveServer(const String &name, int8_t direction) {
	if (direction != -1 && direction != 1) {
		return false;
	}
	std::vector<MediaHubServer> servers = MediaHub_GetServers();
	for (size_t index = 0; index < servers.size(); ++index) {
		if (servers[index].name != name) {
			continue;
		}
		const int nextIndex = static_cast<int>(index) + direction;
		if (nextIndex < 0 || nextIndex >= static_cast<int>(servers.size())) {
			return false;
		}
		std::swap(servers[index], servers[static_cast<size_t>(nextIndex)]);
		return MediaHub_SaveServers(servers);
	}
	return false;
}
