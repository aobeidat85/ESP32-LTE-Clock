// SPDX-License-Identifier: GPL-3.0-only
#include "Esp32LteClock.h"
#include "LteTimeClock.h"
#include <cmath>
#include <cstring>

namespace esp32_lte_time {
namespace {
void serviceClockDecoder() { delay(1); }
bool supported(const Mib& m) {
  return m.valid && m.resourceBlocks==50 && m.ports==4 && !m.extendedPhich && m.phichResource==1;
}
}
bool Esp32LteClock::expired() const { return esp_timer_get_time()>=deadline_||(cellDeadline_&&esp_timer_get_time()>=cellDeadline_); }
void Esp32LteClock::report(ClockStatus s) { status_=s;if(progress_)progress_(s,evidence_); }

bool Esp32LteClock::capture(const uint32_t*& words,uint64_t& stamp,uint64_t target) {
  if(expired())return false;
  auto result=radio_.captureRawView(words,esp32_lte_raw::Receiver::MaxRawSamples,&stamp,target);
  ++evidence_.captures;
  if(result.error!=esp32_lte_raw::Error::None)return false;
  // Re-arm flat ADC captures; never search a sentinel/constant word stream.
  bool varying=false;
  for(unsigned j=5;j<result.count;++j)if((words[j]&0xfffff)!=(words[4]&0xfffff)){varying=true;break;}
  if(!varying){
    radio_.end();delay(2);
    esp32_lte_raw::Config config;config.tuningMHz=activeTuningMHz_;config.gainIndex=config_.gainIndex;
    config.minimalWifiBuffers=true;
    if(!radio_.begin(config))report(ClockStatus::RadioError);
  }
  return varying;
}

bool Esp32LteClock::scheduled(const uint32_t*& words,uint64_t& stamp,unsigned period,
                              unsigned remainder,unsigned subframe,unsigned& capturedSfn) {
  if(expired()||!period||period>512||remainder>=period||subframe>9)return false;
  int64_t now=esp_timer_get_time();
  int64_t n=int64_t(std::ceil((double(now)+100000-phase_-subframe*period_/10)/period_));
  while((sfn_+n)%period!=remainder)++n;
  int64_t target=int64_t(std::llround(phase_+n*period_+subframe*period_/10-8));
  while(target-esp_timer_get_time()>2500000){if(expired())return false;delay(20);}
  capturedSfn=unsigned((sfn_+n)%1024);
  return capture(words,stamp,uint64_t(target));
}

bool Esp32LteClock::acquire(const uint32_t* words,uint64_t stamp,Sync candidate) {
  report(ClockStatus::Acquiring);
  phase_=double(stamp)+candidate.frameOffsetUs;period_=10000.2459;
  carrier_=activeCarrierHz_+candidate.carrierCorrectionHz;
  double firstTime=0;unsigned firstSfn=0;
  for(unsigned attempt=0;attempt<24&&!expired();++attempt){
    Mib mib=rf_.readMib(candidate);
    evidence_.pssScore=candidate.pssScore;evidence_.sssScore=candidate.sssScore;
    evidence_.carrierCorrectionHz=float(carrier_-activeCarrierHz_);
    if(attempt%4==0)report(ClockStatus::Acquiring);
    if(mib.valid){
      ++evidence_.mibs;
      if(!supported(mib)){report(ClockStatus::UnsupportedCell);return false;}
      double observed=double(stamp)+candidate.frameOffsetUs;
      if(firstTime){
        double fitted=0;
        if(fitFramePeriod(firstTime,firstSfn,observed,mib.sfn,fitted)){
          phase_=observed;period_=fitted;sfn_=mib.sfn;
          evidence_.framePeriodUs=period_;evidence_.sfn=sfn_;return true;
        }
      }
      firstTime=observed;firstSfn=mib.sfn;
    }
    // A PSS/SSS phase is enough to target another subframe zero, but not UTC.
    int64_t now=esp_timer_get_time();
    int64_t n=int64_t(std::ceil((double(now)+100000-phase_)/period_));
    uint64_t target=uint64_t(std::llround(phase_+n*period_-8));
    if(!capture(words,stamp,target)){candidate.found=false;continue;}
    candidate=rf_.acquire(words,esp32_lte_raw::Receiver::MaxRawSamples,
                           carrier_-activeTuningMHz_*1000000.0,pci_,0,false);
    if(!candidate.found||candidate.subframe!=0)continue;
    phase_=double(stamp)+candidate.frameOffsetUs;
    carrier_+=candidate.carrierCorrectionHz;
  }
  return false;
}

bool Esp32LteClock::track() {
  for(unsigned attempt=0;attempt<6&&!expired();++attempt){
    const uint32_t* words=nullptr;uint64_t stamp=0;unsigned expected=0;
    unsigned sf=attempt%2?5:0;
    if(!scheduled(words,stamp,2,0,sf,expected))continue;
    Sync sync=rf_.acquire(words,esp32_lte_raw::Receiver::MaxRawSamples,
                          carrier_-activeTuningMHz_*1000000.0,pci_,0,false);
    if(!sync.found||sync.subframe!=sf)continue;
    double observed=double(stamp)+sync.frameOffsetUs;
    int64_t n=int64_t(std::llround((observed-phase_)/period_));
    if(std::fabs(observed-(phase_+n*period_))>80)continue;
    Mib mib=rf_.readMib(sync);
    if(mib.valid){
      ++evidence_.mibs;
      if(!supported(mib)||mib.sfn!=expected)return false;
    }
    phase_=observed;sfn_=uint16_t(expected);carrier_+=sync.carrierCorrectionHz;
    evidence_.sfn=sfn_;return true;
  }
  return false;
}

bool Esp32LteClock::receiveTime(const uint32_t* words,uint64_t stamp,Sync found) {
  if(!acquire(words,stamp,found))return false;
  uint8_t payload[MaxTransportBits/8];
  SiSchedule schedule;bool hasSchedule=false;
  for(unsigned attempt=0;attempt<10&&!expired();++attempt){
    report(ClockStatus::ReadingSchedule);
    // A recent CRC-fitted anchor can still target a CRC-checked SI window
    // when a tracking snapshot fades. Abandon it after a bounded interval.
    if(!track()&&esp_timer_get_time()-int64_t(phase_)>10000000)break;
    unsigned frame=0;
    if(!scheduled(words,stamp,2,0,5,frame))continue;
    size_t bytes=0;
    auto r=rf_.readSystem(words,esp32_lte_raw::Receiver::MaxRawSamples,
        carrier_-activeTuningMHz_*1000000.0,pci_,5,payload,bytes);
    if(r!=Result::Ok)continue;
    r=parseSib1Schedule(payload,bytes,schedule);
    if(r==Result::NoTime){report(ClockStatus::UnsupportedCell);break;}
    if(r==Result::Ok){evidence_.cellId=schedule.cellId;hasSchedule=true;break;}
  }
  if(!hasSchedule)return false;
  for(unsigned attempt=0;attempt<16&&!expired();++attempt){
    report(ClockStatus::ReadingTime);
    if(!track()&&esp_timer_get_time()-int64_t(phase_)>10000000)break;
    unsigned frame=0,position=schedule.startMs;
    if(!scheduled(words,stamp,schedule.periodFrames,position/10,position%10,frame))continue;
    size_t bytes=0;
    auto r=rf_.readSystem(words,esp32_lte_raw::Receiver::MaxRawSamples,
        carrier_-activeTuningMHz_*1000000.0,pci_,position%10,payload,bytes);
    Sib8Time time;UtcTime utc;
    if(r!=Result::Ok||parseSib8(payload,bytes,time)!=Result::Ok)continue;
    if(toUtc(time,utc)!=Result::Ok){report(ClockStatus::ConversionError);return false;}
    int64_t reference=0;
    if(sib8ReferenceUptime(schedule,frame,position%10,stamp,rf_.timingStart,period_,reference)!=Result::Ok)continue;
    evidence_.rawTicks=time.ticks;evidence_.referenceUptimeUs=reference;
    evidence_.utcAtReferenceUs=utc.unixUs;evidence_.timingSample=rf_.timingStart;
    evidence_.timePayloadBytes=uint8_t(bytes);
    std::memcpy(evidence_.timePayload,payload,bytes);
    // Decode delay is accounted for by the same boot's monotonic clock.
    if(expired()||!setUtcFromBoundary(utc,reference,config_.maxObservationAgeUs))continue;
    report(ClockStatus::Synced);return true;
  }
  return false;
}

bool Esp32LteClock::sync(uint32_t timeoutMs,const ClockConfig& config,ClockProgress progress) {
  radio_.end();config_=config;progress_=progress;evidence_={};cellDeadline_=0;
  if(!timeoutMs||timeoutMs>3600000||!config.maxObservationAgeUs||
     !config.capturesPerFrequency||config.capturesPerFrequency>32||
     !config.cellTimeoutMs||config.cellTimeoutMs>3600000||
     !config.cellAttempts||config.cellAttempts>8||
     (!config.autoScan&&(!std::isfinite(config.carrierHz)||
       std::fabs(config.carrierHz-config.tuningMHz*1000000.0)>6000000||
       config.pcis[0]>503||config.pcis[1]>503))){
    report(ClockStatus::RadioError);return false;
  }
  deadline_=esp_timer_get_time()+int64_t(timeoutMs)*1000;
  auto selectFrequency=[&](){
    const uint16_t f=CellScanPlan::frequency(scanCursor_);
    activeCarrierHz_=config.autoScan?double(f)*100000:config.carrierHz;
    activeTuningMHz_=config.autoScan?CellScanPlan::tuningMHz(f):config.tuningMHz;
    evidence_.carrier100kHz=uint16_t(std::llround(activeCarrierHz_/100000));
    evidence_.tuningMHz=activeTuningMHz_;++evidence_.scanVisits;
    evidence_.pci=65535;evidence_.sfn=0;evidence_.cellId=0;
    evidence_.pssScore=evidence_.sssScore=evidence_.carrierCorrectionHz=0;
  };
  selectFrequency();
  esp32_lte_raw::Config radioConfig;
  radioConfig.tuningMHz=activeTuningMHz_;radioConfig.gainIndex=config.gainIndex;
  radioConfig.minimalWifiBuffers=true;
  if(!rf_.reserve()){evidence_.allocationFailure=rf_.allocationFailure;report(ClockStatus::NoMemory);return false;}
  if(!radio_.begin(radioConfig)){
    evidence_.radioError=uint8_t(radio_.lastError());rf_.release();report(ClockStatus::RadioError);return false;
  }
  rf_.service=serviceClockDecoder;
  rf_.cancelled=[](void* context){return static_cast<Esp32LteClock*>(context)->expired();};
  rf_.cancelContext=this;
  unsigned dwell=0,excludedCount=0,attemptedCount=0;
  uint16_t excluded[32]={};
  uint16_t attemptedPci[32]={};
  uint8_t attempts[32]={};
  while(esp_timer_get_time()<deadline_){
    cellDeadline_=0;
    if(config.autoScan&&dwell>=config.capturesPerFrequency){
      scanCursor_=(scanCursor_+1)%CellScanPlan::count();selectFrequency();
      dwell=0;excludedCount=0;attemptedCount=0;
      std::memset(attempts,0,sizeof(attempts));
      if(!radio_.tune(activeTuningMHz_)){
        evidence_.radioError=uint8_t(radio_.lastError());
        radio_.end();rf_.release();report(ClockStatus::RadioError);return false;
      }
    }
    report(ClockStatus::Searching);
    const uint32_t* words=nullptr;uint64_t stamp=0;
    ++dwell;
    if(!capture(words,stamp))continue;
    for(unsigned candidateIndex=0;candidateIndex<(config.autoScan?1u:2u)&&!expired();++candidateIndex){
      Sync found;
      if(config.autoScan){
        found=rf_.discover(words,esp32_lte_raw::Receiver::MaxRawSamples,
                          activeCarrierHz_-activeTuningMHz_*1000000.0,excluded,excludedCount);
      }else{
        found=rf_.acquire(words,esp32_lte_raw::Receiver::MaxRawSamples,
                         activeCarrierHz_-activeTuningMHz_*1000000.0,config.pcis[candidateIndex],0,true);
      }
      evidence_.rawStable=rf_.rawStable;
      evidence_.pssScore=rf_.pssPeak;evidence_.sssScore=rf_.sssPeak;
      if(!found.found)continue;
      pci_=found.pci;evidence_.pci=pci_;++evidence_.discoveredCells;
      unsigned slot=0;
      for(;slot<attemptedCount;++slot)if(attemptedPci[slot]==pci_)break;
      if(slot==attemptedCount&&attemptedCount<32)attemptedPci[attemptedCount++]=pci_;
      if(slot<32)++attempts[slot];
      evidence_.cellId=0;
      evidence_.pssScore=found.pssScore;evidence_.sssScore=found.sssScore;
      cellDeadline_=esp_timer_get_time()+int64_t(config.cellTimeoutMs)*1000;
      if(receiveTime(words,stamp,found)){
        cellDeadline_=0;radio_.end();rf_.release();return true;
      }
      ++evidence_.rejectedCells;
      if(config.autoScan){
        const bool reject=status_==ClockStatus::UnsupportedCell||slot>=32||
                          attempts[slot]>=config.cellAttempts;
        if(reject){if(excludedCount<32)excluded[excludedCount++]=pci_;}
        else dwell=0; // Fade/CRC failure: bounded fresh discovery on this carrier.
      }
      cellDeadline_=0;
      break; // Acquisition replaced the borrowed raw sample lease.
    }
    delay(13);delayMicroseconds(esp_random()%5000);
  }
  // Resume at the next frequency after a timed-out dwell on a later sync().
  if(config.autoScan)scanCursor_=(scanCursor_+1)%CellScanPlan::count();
  cellDeadline_=0;radio_.end();rf_.release();report(ClockStatus::TimedOut);return false;
}
} // namespace esp32_lte_time
