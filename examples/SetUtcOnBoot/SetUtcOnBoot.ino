// SPDX-License-Identifier: GPL-3.0-only
#include <Esp32LteClock.h>
#include <time.h>

static esp32_lte_time::Esp32LteClock lteClock;

static void progress(esp32_lte_time::ClockStatus status,
                     const esp32_lte_time::ClockEvidence& evidence) {
  static uint32_t lastVisit = 0;
  static auto lastStatus = esp32_lte_time::ClockStatus::Idle;
  if (evidence.scanVisits != lastVisit || status != lastStatus) {
    Serial.printf("LTE boot_ms=%u status=%u carrier_kHz=%u PCI=%u captures=%u MIBs=%u\n",
        millis(), unsigned(status), evidence.carrier100kHz * 100u,
        evidence.pci, evidence.captures, evidence.mibs);
    lastVisit = evidence.scanVisits;
    lastStatus = status;
  }
}

void setup() {
  Serial.begin(115200); // Diagnostics only. No serial input is required.
  Serial.println("Receiving UTC from LTE using the onboard antenna...");
  // Default configuration discovers both frequency and PCI automatically.
  while (!lteClock.sync(600000, esp32_lte_time::ClockConfig(), progress)) {
    Serial.println("No verified LTE time yet. Retrying in five seconds.");
    delay(5000);
  }
  const auto& evidence = lteClock.evidence();
  Serial.printf("UTC synchronized. boot_ms=%u carrier_kHz=%u PCI=%u cell=%u captures=%u MIBs=%u ticks=%llu\n",
      millis(), evidence.carrier100kHz * 100u, evidence.pci, evidence.cellId, evidence.captures, evidence.mibs,
      (unsigned long long)evidence.rawTicks);
}

void loop() {
  time_t now = time(nullptr);
  tm utc;
  gmtime_r(&now, &utc);
  char text[32];
  strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%SZ", &utc);
  Serial.println(text);
  delay(1000);
}
