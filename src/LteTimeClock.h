// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "LteTimeDecoder.h"
#include "esp_timer.h"
#include <sys/time.h>

namespace esp32_lte_time {
// Integration hook for a future LIVE acquisition path. The caller must supply
// the device uptime corresponding to the SIB8 SI-window reference boundary,
// derived from this boot's LTE frame clock. Never pass a replay observation.
// A false return leaves the caller responsible for reporting failure to sync.
inline bool setUtcFromBoundary(const UtcTime& utc, int64_t boundaryUptimeUs,
                               uint64_t maxAgeUs = 60000000) {
  int64_t now = 0;
  if (advanceToNow(utc, boundaryUptimeUs, esp_timer_get_time(), maxAgeUs, now) != Result::Ok)
    return false;
  timeval tv;
  tv.tv_sec = static_cast<time_t>(now / 1000000);
  if (int64_t(tv.tv_sec) != now / 1000000) return false;
  tv.tv_usec = static_cast<suseconds_t>(now % 1000000);
  return settimeofday(&tv, nullptr) == 0;
}
} // namespace esp32_lte_time
