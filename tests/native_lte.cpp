// SPDX-License-Identifier: GPL-3.0-only
#include "LteTimeDecoder.h"
#include "LteRfDecoder.h"
#include "LteCellScan.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>
using namespace esp32_lte_time;
static TransportDecoder decoder;
static RfDecoder rf;
int main(int argc, char** argv) {
  if(argc==3&&!std::strcmp(argv[1],"siwindow")){
    unsigned window=std::atoi(argv[2]);std::printf("[");
    for(unsigned i=0;i<(window&&window<=40?(2*window-1)*2:1);++i)
      std::printf("%s%d",i?",":"",siWindowOffset(window,i));
    std::printf("]\n");return 0;
  }
  if(argc==2&&!std::strcmp(argv[1],"scanplan")){
    std::printf("[");
    for(unsigned i=0;i<CellScanPlan::count();++i){
      const auto f=CellScanPlan::frequency(i);
      std::printf("%s[%u,%u]",i?",":"",f,CellScanPlan::tuningMHz(f));
    }
    std::printf("]\n");return 0;
  }
  if(argc==6&&!std::strcmp(argv[1],"fit")){
    double period=0;
    bool ok=fitFramePeriod(std::atof(argv[2]),std::atoi(argv[3]),std::atof(argv[4]),std::atoi(argv[5]),period);
    std::printf("{\"ok\":%s,\"period\":%.12g}\n",ok?"true":"false",period);return 0;
  }
  if(argc==10&&!std::strcmp(argv[1],"reference")){
    SiSchedule schedule;schedule.hasSib8=true;schedule.periodFrames=uint16_t(std::atoi(argv[2]));
    schedule.startMs=uint16_t(std::atoi(argv[3]));schedule.windowMs=uint8_t(std::atoi(argv[4]));
    int64_t value=0;
    auto r=sib8ReferenceUptime(schedule,std::atoi(argv[5]),std::atoi(argv[6]),std::strtoull(argv[7],nullptr,10),
                              std::atoi(argv[8]),std::atof(argv[9]),value);
    std::printf("{\"status\":%u,\"reference\":%lld}\n",unsigned(r),(long long)value);return 0;
  }
  if (argc >= 6 && (!std::strcmp(argv[1], "rf") || !std::strcmp(argv[1], "sync") || !std::strcmp(argv[1], "discover"))) {
    std::ifstream file(argv[2], std::ios::binary);
    std::vector<uint8_t> input{std::istreambuf_iterator<char>(file), {}};
    if (!file || input.size()%4) return 3;
    std::vector<uint32_t> raw(input.size()/4); std::memcpy(raw.data(),input.data(),input.size());
    double offset=std::strtod(argv[3],nullptr); unsigned pci=std::atoi(argv[4]);
    if (!std::strcmp(argv[1], "sync") || !std::strcmp(argv[1], "discover")) {
      uint16_t excluded=uint16_t(pci);
      if(argc>=7&&std::atoi(argv[6])<0)rf.cancelled=[](void*){return true;};
      auto s=!std::strcmp(argv[1], "discover")?
        rf.discover(raw.data(),raw.size(),offset,pci<=503?&excluded:nullptr,pci<=503?1:0):
        rf.acquire(raw.data(),raw.size(),offset,pci,float(std::atof(argv[5])),argc<7||std::atoi(argv[6]));
      auto m=rf.readMib(s);
      std::printf("{\"pci\":%u,\"found\":%s,\"pss\":%.7g,\"sss\":%.7g,\"start\":%u,\"sf\":%u,\"cfo\":%.7g,"
        "\"frame_offset_us\":%.10g,\"mib\":%s,\"sfn\":%u,\"nrb\":%u,\"ports\":%u,\"ng\":%.7g}\n",
        s.pci,s.found?"true":"false",s.pssScore,s.sssScore,s.pssStart,s.subframe,s.carrierCorrectionHz,
        s.frameOffsetUs,m.valid?"true":"false",m.sfn,m.resourceBlocks,m.ports,m.phichResource);
      return 0;
    }
    uint8_t payload[MaxTransportBits/8];size_t bytes=0;
    auto r=rf.readSystem(raw.data(),raw.size(),offset,pci,std::atoi(argv[5]),payload,bytes);
    std::printf("{\"status\":%u,\"payload\":\"",unsigned(r));
    if(r==Result::Ok)for(size_t i=0;i<bytes;++i)std::printf("%02x",payload[i]);
    Sib8Time t;UtcTime u;SiSchedule schedule;
    auto tr=r==Result::Ok?parseSib8(payload,bytes,t):r;
    if(tr==Result::Ok)tr=toUtc(t,u);
    auto sr=r==Result::Ok?parseSib1Schedule(payload,bytes,schedule):r;
    std::printf("\",\"time_status\":%u,\"ticks\":%llu,\"unix_us\":%lld,\"schedule_status\":%u,"
      "\"cell\":%u,\"period\":%u,\"window\":%u,\"window_start\":%u,\"timing\":%d,\"cp_score\":%.7g,\"grants\":%u,\"workspace\":%zu}\n",
      unsigned(tr),(unsigned long long)t.ticks,(long long)u.unixUs,unsigned(sr),schedule.cellId,
      schedule.periodFrames,schedule.windowMs,schedule.startMs,rf.timingStart,rf.cpScore,rf.grants,rf.workspaceBytes());
    return 0;
  }
  if (argc == 2 && !std::strcmp(argv[1], "memory")) {
    std::printf("%zu\n", sizeof(decoder)); return 0;
  }
  if (argc < 3) return 2;
  Result r = Result::InvalidInput;
  uint8_t payload[MaxTransportBits / 8] = {};
  size_t bytes = 0;
  bool transport = false;
  Sib8Time time;
  if (!std::strcmp(argv[1], "utc") && argc == 4) {
    time.ticks = std::strtoull(argv[2], nullptr, 10); time.width = uint8_t(std::atoi(argv[3]));
    r = Result::Ok;
  } else if (!std::strcmp(argv[1], "advance") && argc == 6) {
    UtcTime at; at.unixUs = std::strtoll(argv[2], nullptr, 10);
    int64_t now = 0;
    r = advanceToNow(at, std::strtoll(argv[3], nullptr, 10), std::strtoll(argv[4], nullptr, 10),
                     std::strtoull(argv[5], nullptr, 10), now);
    std::printf("{\"status\":%u,\"unix_us\":%lld}\n", unsigned(r), (long long)now); return 0;
  } else {
    std::ifstream f(argv[2], std::ios::binary);
    if (!f) return 3;
    std::vector<uint8_t> input{std::istreambuf_iterator<char>(f), {}};
    if (!std::strcmp(argv[1], "rrc")) {
      r = parseSib8(input.data(), input.size(), time);
    } else if ((!std::strcmp(argv[1], "turbo") && argc == 5) ||
               ((!std::strcmp(argv[1], "control") || !std::strcmp(argv[1], "control_fast")) && argc == 4)) {
      if (input.size() % sizeof(float)) return 4;
      std::vector<float> llr(input.size() / sizeof(float));
      std::memcpy(llr.data(), input.data(), input.size());
      bytes = std::atoi(argv[3]);
      if (!std::strcmp(argv[1], "control") || !std::strcmp(argv[1], "control_fast")) {
        uint8_t bits[64] = {};
        r = decodeSiControl(llr.data(), llr.size(), bytes, bits, sizeof(bits),nullptr,0xffff,
                            !std::strcmp(argv[1], "control_fast"));
        std::printf("{\"status\":%u,\"bits\":\"", unsigned(r));
        if (r == Result::Ok) for (size_t i = 0; i < bytes; ++i) std::printf("%u", bits[i]);
        std::puts("\"}"); return 0;
      }
      r = decoder.decode(llr.data(), llr.size(), std::atoi(argv[4]), bytes, payload, sizeof(payload));
      transport = true;
      bytes /= 8;
      std::printf("{\"transport_status\":%u,\"payload\":\"", unsigned(r));
      if (r == Result::Ok) for (size_t i = 0; i < bytes; ++i) std::printf("%02x", payload[i]);
      std::printf("\",\"rrc\":");
      if (r == Result::Ok) r = parseSib8(payload, bytes, time);
    }
  }
  UtcTime utc;
  Result conversion = r == Result::Ok ? toUtc(time, utc) : r;
  std::printf("{\"status\":%u,\"ticks\":%llu,\"width\":%u,\"synchronized\":%s,"
              "\"utc_status\":%u,\"unix_us\":%lld}%s\n", unsigned(r),
              (unsigned long long)time.ticks, time.width, time.eutraSynchronized ? "true" : "false",
              unsigned(conversion), (long long)utc.unixUs, transport ? "}" : "");
  return 0;
}
