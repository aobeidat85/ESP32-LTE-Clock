// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <stdint.h>

namespace esp32_lte_time {
// Union of reachable FDD downlink ranges for bands 1/2/3/4/7/9/10/23/25/30
// and their overlapping extensions. Units: 100 kHz. This is a search plan,
// not a claim of RF sensitivity or time-message support across these bands.
struct ScanRange { uint16_t first, last; };
class CellScanPlan {
 public:
  static constexpr unsigned rangeCount() { return 5; }
  static ScanRange range(unsigned i) {
    const ScanRange ranges[]={{21100,21999},{19300,19949},{18420,18799},
                              {23500,23599},{26200,26899}};
    return i<rangeCount()?ranges[i]:ScanRange{0,0};
  }
  static unsigned count() {
    unsigned n=0;
    for(unsigned i=0;i<rangeCount();++i){auto r=range(i);n+=r.last-r.first+1;}
    return n;
  }
  // First pass: 5 MHz centers, from the middle of each range outward. Then
  // every remaining 100 kHz raster point. No frequency or PCI learned at the
  // bench is stored. All raster points occur exactly once per complete pass.
  static uint16_t frequency(unsigned index) {
    index%=count();
    for(unsigned i=0;i<rangeCount();++i){
      auto r=range(i);const int first=(r.first+49)/50,last=r.last/50;
      const int center=(first+last+1)/2;
      for(int step=0;step<=2*(last-first+1);++step){
        const int p=center+(step%2?-(step+1)/2:step/2);
        if(p<first||p>last)continue;
        if(index--==0)return uint16_t(p*50);
      }
    }
    for(unsigned i=0;i<rangeCount();++i){
      auto r=range(i);
      for(unsigned f=r.first;f<=r.last;++f){
        if(f%50==0)continue;
        if(index--==0)return uint16_t(f);
      }
    }
    return 0;
  }
  static uint16_t tuningMHz(uint16_t carrier100kHz) {
    const unsigned lo=carrier100kHz/10-1;
    return uint16_t(lo<1842?1842:lo);
  }
};
} // namespace esp32_lte_time
