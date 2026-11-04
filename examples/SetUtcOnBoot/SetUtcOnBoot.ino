// SPDX-License-Identifier: GPL-3.0-only
#include <Esp32LteClock.h>
#include <time.h>

static esp32_lte_time::Esp32LteClock lteClock;

void setup() {
  Serial.begin(115200); // Diagnostics only. No serial input is required.
  Serial.println("Receiving UTC from LTE using the onboard antenna...");
  while (!lteClock.sync(180000)) {
    Serial.println("No verified LTE time yet. Retrying in five seconds.");
    delay(5000);
  }
  const auto& evidence = lteClock.evidence();
  Serial.printf("UTC synchronized. PCI=%u cell=%u captures=%u MIBs=%u ticks=%llu\n",
      evidence.pci, evidence.cellId, evidence.captures, evidence.mibs,
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
