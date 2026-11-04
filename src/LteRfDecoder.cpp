// SPDX-License-Identifier: GPL-3.0-only
// Portable local DSP port of experiments/satellite/lte_{sync,pbch,downlink,window_read}.py.
// Initial supported downlink profile is deliberately bounded; no UE traffic.
#include "LteRfDecoder.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>
#if defined(ARDUINO_ARCH_ESP32)
#include "esp_heap_caps.h"
#endif

namespace esp32_lte_time {
namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr float Sqrt2 = 1.4142135623730951f;
#include "LteRfFir.inc"
Complex phase(double angle) { return {float(std::cos(angle)),float(std::sin(angle))}; }
Complex interpolate(Complex a, Complex b, float f) { return a + (b-a)*f; }
Complex signedIq(uint32_t w) {
  int i = (w >> 10) & 1023, q = w & 1023;
  return {float(i >= 512 ? i-1024 : i), float(q >= 512 ? q-1024 : q)};
}
void fft(Complex* x, unsigned n, bool inverse = false) {
  for (unsigned i=1,j=0; i<n; ++i) {
    unsigned bit=n>>1;
    for (; j&bit; bit>>=1) j^=bit;
    j^=bit;
    if (i<j) std::swap(x[i],x[j]);
  }
  for (unsigned length=2; length<=n; length<<=1) {
    Complex step=phase((inverse ? 2 : -2)*Pi/length);
    for (unsigned i=0; i<n; i+=length) {
      Complex w{1,0};
      for (unsigned j=0; j<length/2; ++j) {
        Complex a=x[i+j], b=x[i+j+length/2]*w;
        x[i+j]=a+b; x[i+j+length/2]=a-b; w=w*step;
      }
    }
  }
  if (inverse) for (unsigned i=0; i<n; ++i) x[i]=x[i]/float(n);
}
struct Gold {
  uint32_t x=1,y;
  explicit Gold(uint32_t seed) : y(seed) { for (unsigned i=0;i<1600;++i) bit(); }
  unsigned bit() {
    unsigned value=(x^y)&1;
    x=(((x^(x>>3))&1)<<30)|(x>>1);
    y=(((y^(y>>1)^(y>>2)^(y>>3))&1)<<30)|(y>>1);
    return value;
  }
  float sign() { return bit() ? -1.f : 1.f; }
};
void pss(unsigned nid2, Complex* sequence) {
  const unsigned roots[3]={25,29,34};
  for (unsigned j=0;j<62;++j) {
    unsigned n=j+(j>=31);
    sequence[j]=phase(-Pi*roots[nid2]*n*(n+1)/63);
  }
}
void sss(unsigned pci, unsigned half, float* sequence) {
  int x[3][31]={};
  const unsigned taps[3]={5,9,23};
  for (unsigned a=0;a<3;++a) {
    x[a][4]=1;
    for (unsigned n=0;n<26;++n) {
      unsigned b=0;
      for (unsigned t=0;t<5;++t) if ((taps[a]>>t)&1) b^=x[a][n+t];
      x[a][n+5]=b;
    }
    for (int& v:x[a]) v=1-2*v;
  }
  unsigned nid1=pci/3,nid2=pci%3,qp=nid1/30,q=(nid1+qp*(qp+1)/2)/30;
  unsigned mp=nid1+q*(q+1)/2,m0=mp%31,m1=(m0+mp/31+1)%31;
  unsigned a=half ? m1:m0,b=half ? m0:m1;
  for (unsigned n=0;n<31;++n) {
    sequence[2*n]=float(x[0][(n+a)%31]*x[1][(n+nid2)%31]);
    sequence[2*n+1]=float(x[0][(n+b)%31]*x[1][(n+nid2+3)%31]*x[2][(n+a%8)%31]);
  }
}
unsigned syncBin(unsigned j) { return j<31 ? 97+j : j-30; }
float median(float* x, size_t n) {
  if (!n) return 1e-12f;
  std::nth_element(x,x+n/2,x+n);
  float result=x[n/2];
  if (!(n&1)) result=(result+*std::max_element(x,x+n/2))*.5f;
  return std::max(result,1e-12f);
}
unsigned readBits(const uint8_t* b,unsigned first,unsigned n) {
  unsigned value=0;
  while (n--) value=(value<<1)|b[first++];
  return value;
}
uint32_t fingerprint(const uint32_t* raw,size_t count) {
  uint32_t h=2166136261u;
  for(size_t i=0;i<count;++i)h=(h^static_cast<const volatile uint32_t*>(raw)[i])*16777619u;
  return h;
}
struct Bits {
  const uint8_t* p; size_t length,position=0; bool good=true;
  Bits(const uint8_t* data,size_t n):p(data),length(n*8){}
  unsigned get(unsigned n) {
    if(n>32 || position+n>length){good=false;return 0;}
    unsigned v=0;
    while(n--){v=(v<<1)|((p[position/8]>>(7-position%8))&1);++position;}
    return v;
  }
  unsigned constrained(unsigned lo,unsigned hi) {
    unsigned r=hi-lo,n=0;
    while(r){++n;r>>=1;}
    unsigned v=get(n);
    if(v>hi-lo)good=false;
    return v+lo;
  }
};
bool dataRe(unsigned sym,unsigned sc,unsigned pci,unsigned sf,unsigned cfi,unsigned first,unsigned rbs) {
  if(sym<cfi || sc<12*first || sc>=12*(first+rbs))return false;
  unsigned l=sym%7;
  if((l==0||l==1||l==4)&&sc%3==pci%3)return false;
  if((sf==0||sf==5)&&(sym==5||sym==6)&&sc>=264&&sc<336)return false;
  if(sf==0&&sym>=7&&sym<=10&&sc>=264&&sc<336)return false;
  return true;
}
} // namespace

bool RfDecoder::reserve() {
  if(sampleBanks_[0])return true;
  // Several separately checked allocations fit the ESP32's multiple SRAM
  // regions without overflowing its much smaller static DRAM linker segment.
  allocationFailure=1;
#if defined(ARDUINO_ARCH_ESP32)
  sampleBanks_[0]=static_cast<uint32_t*>(heap_caps_malloc(IramWords*4,MALLOC_CAP_EXEC|MALLOC_CAP_32BIT));
  // The capture aperture's IRAM alias begins at 0x400a8000. Do not borrow it.
  if(sampleBanks_[0]&&(reinterpret_cast<uintptr_t>(sampleBanks_[0])<0x40080000u||
      reinterpret_cast<uintptr_t>(sampleBanks_[0])+IramWords*4>0x400a8000u)){
    heap_caps_free(sampleBanks_[0]);sampleBanks_[0]=nullptr;
  }
#else
  sampleBanks_[0]=new(std::nothrow) uint32_t[IramWords];
#endif
  if(!sampleBanks_[0]){release();return false;}
  sampleBanks_[1]=new(std::nothrow) uint32_t[StorageWords-IramWords];
  if(!sampleBanks_[1]){release();return false;}
  allocationFailure=2;turbo_=new(std::nothrow) TransportDecoder;if(!turbo_){release();return false;}
  allocationFailure=3;pilots_=new(std::nothrow) Complex[4][4][100];if(!pilots_){release();return false;}
  allocationFailure=4;fft_=new(std::nothrow) Complex[2048];if(!fft_){release();return false;}
  allocationFailure=5;powers_=new(std::nothrow) float[PowerCapacity];if(!powers_){release();return false;}
  allocationFailure=6;control_=new(std::nothrow) float[16*72];if(!control_){release();return false;}
  allocationFailure=7;ring_=new(std::nothrow) Complex[256];if(!ring_){release();return false;}
  allocationFailure=0;
  return true;
}
void RfDecoder::release() {
#if defined(ARDUINO_ARCH_ESP32)
  heap_caps_free(sampleBanks_[0]);
#else
  delete[] sampleBanks_[0];
#endif
  delete[] sampleBanks_[1];delete[] fft_;delete[] ring_;delete[] pilots_;delete[] powers_;delete[] control_;delete turbo_;
  sampleBanks_[0]=sampleBanks_[1]=nullptr;fft_=ring_=nullptr;pilots_=nullptr;powers_=control_=nullptr;turbo_=nullptr;
  timeCount_=narrowCount_=0;
}

uint32_t RfDecoder::loadWord(size_t index) const {
  // Volatile is intentional: Xtensa must issue L32I, not an FPU load from IRAM.
  const volatile uint32_t* p=index<IramWords?sampleBanks_[0]+index:sampleBanks_[1]+index-IramWords;
  return *p;
}
void RfDecoder::storeWord(size_t index,uint32_t value) {
  volatile uint32_t* p=index<IramWords?sampleBanks_[0]+index:sampleBanks_[1]+index-IramWords;
  *p=value;
}
Complex RfDecoder::loadComplex(size_t index) const {
  union Word { uint32_t u; float f; } a,b;
  a.u=loadWord(index);b.u=loadWord(index+1);return {a.f,b.f};
}
void RfDecoder::storeComplex(size_t index,Complex value) {
  union Word { uint32_t u; float f; } a,b;
  a.f=value.r;b.f=value.i;storeWord(index,a.u);storeWord(index+1,b.u);
}
Complex RfDecoder::timeSample(size_t index) const {
  uint32_t word=loadWord(index);
  return {int16_t(word&65535)/16.f,int16_t(word>>16)/16.f};
}

int siWindowOffset(unsigned windowMs,unsigned attempt) {
  if(!windowMs||windowMs>40)return -1;
  const unsigned step=attempt%(2*windowMs-1);
  unsigned index=step%2?(step+1)/2:0;
  for(unsigned subframe=0;subframe<10;++subframe)
    for(unsigned offset=subframe;offset<windowMs;offset+=10)
      if(index--==0)return int(offset);
  return -1;
}

bool fitFramePeriod(double first,unsigned firstSfn,double last,unsigned lastSfn,double& period) {
  if(!std::isfinite(first)||!std::isfinite(last)||first<0||last<=first||last-first>60000000||
      firstSfn>=1024||lastSfn>=1024)return false;
  int64_t n=int64_t(std::llround((last-first)/10000.2459));
  if(n<=0||(firstSfn+n)%1024!=lastSfn)return false;
  double fitted=(last-first)/double(n);
  if(fitted<=9999||fitted>=10001)return false;
  period=fitted;return true;
}

Result sib8ReferenceUptime(const SiSchedule& schedule,unsigned capturedSfn,unsigned subframe,
                            uint64_t stamp,int timing,double period,int64_t& reference) {
  if(!schedule.hasSib8||!schedule.periodFrames||schedule.periodFrames>512||capturedSfn>=1024||
     subframe>9||!schedule.windowMs||schedule.startMs+schedule.windowMs>schedule.periodFrames*10||
     !std::isfinite(period)||period<=9999||period>=10001||timing < -80||timing>320||
     stamp>uint64_t(INT64_MAX)-1000000)return Result::InvalidReference;
  unsigned position=(capturedSfn%schedule.periodFrames)*10+subframe;
  if(position<schedule.startMs||position>=schedule.startMs+schedule.windowMs)return Result::InvalidReference;
  unsigned boundary=(schedule.startMs+schedule.windowMs+9)/10;
  int framesAhead=int(boundary)-int(capturedSfn%schedule.periodFrames);
  if(framesAhead<0||framesAhead>5)return Result::InvalidReference;
  double frameStart=double(stamp)+timing*period/153600.0-subframe*period/10;
  double result=frameStart+framesAhead*period;
  if(result<0)return Result::InvalidReference;
  reference=int64_t(std::llround(result));return Result::Ok;
}

Result parseSib1Schedule(const uint8_t* payload,size_t bytes,SiSchedule& schedule) {
  schedule={};
  if(!payload||!bytes||bytes>MaxTransportBits/8)return Result::InvalidInput;
  Bits b(payload,bytes);
  if(b.get(1)!=0||b.get(1)!=1)return Result::Unsupported;
  bool pmax=b.get(1),tdd=b.get(1);b.get(1); // nonCriticalExtension follows root fields.
  bool csg=b.get(1);
  unsigned plmns=b.constrained(1,6);
  for(unsigned i=0;i<plmns&&b.good;++i){
    bool mcc=b.get(1);
    if(mcc)for(unsigned j=0;j<3;++j)b.constrained(0,9);
    unsigned digits=b.constrained(2,3);
    for(unsigned j=0;j<digits;++j)b.constrained(0,9);
    b.get(1);
  }
  b.get(16);schedule.cellId=b.get(28);
  b.get(1);b.get(1);b.get(1);if(csg)b.get(27);
  bool rxOffset=b.get(1);b.constrained(0,48);if(rxOffset)b.constrained(1,8);
  if(pmax)b.constrained(0,63);
  schedule.band=uint8_t(b.constrained(1,64));
  unsigned entries=b.constrained(1,32);
  unsigned selected=0;
  for(unsigned i=0;i<entries&&b.good;++i){
    unsigned periodicity=b.constrained(0,6),types=b.constrained(0,31);
    bool hasTime=false;
    for(unsigned j=0;j<types&&b.good;++j){
      bool extension=b.get(1);
      if(extension){if(b.get(1))return Result::Unsupported;b.get(6);}
      else if(b.get(4)==5)hasTime=true; // root enum sibType8.
    }
    if(hasTime){
      if(schedule.hasSib8)return Result::MalformedRrc;
      schedule.hasSib8=true;schedule.periodFrames=uint16_t(8u<<periodicity);selected=i;
    }
  }
  if(tdd){b.constrained(0,6);b.constrained(0,8);}
  const uint8_t windows[7]={1,2,5,10,15,20,40};
  unsigned w=b.constrained(0,6);
  if(!b.good||w>=7)return Result::MalformedRrc;
  schedule.windowMs=windows[w];schedule.startMs=uint16_t(selected*schedule.windowMs);
  b.get(5);
  if(!b.good)return Result::MalformedRrc;
  if(tdd)return Result::Unsupported;
  // Later noncritical extensions cannot alter these Release-8 root fields.
  if(!schedule.hasSib8)return Result::NoTime;
  if(schedule.startMs+schedule.windowMs>schedule.periodFrames*10)return Result::MalformedRrc;
  return Result::Ok;
}

bool RfDecoder::resample(const uint32_t* raw,size_t count,double offset,bool narrow) {
  if(!reserve())return false;
  if(!raw||count<12000||count>16380||!std::isfinite(offset)||std::fabs(offset)>7000000)return false;
  bool varying=false;
  for(size_t j=5;j<count;++j)if((raw[j]&0xfffff)!=(raw[4]&0xfffff)){varying=true;break;}
  if(!varying)return false;
  size_t first=narrow?4:0;
  size_t n=narrow?std::min(size_t(13400),count-first):count;
  unsigned up=narrow?3:24;
  Complex mean{0,0};
  for(size_t i=0;i<n;++i)mean=mean+signedIq(raw[first+i]);
  mean=mean/float(n);
  size_t outputs=(n*up+24)/25;
  if(narrow&&outputs>2048)return false;
  Complex step=phase(-2*Pi*offset/16000000),rotation{1,0};
  size_t filled=0;
  for(size_t m=0;m<outputs;++m){
    if(m%512==0&&stopping())return false;
    if(service&&m%512==0)service();
    int center=int(m*25+250),last=center/int(up);
    while(filled<n&&filled<=size_t(last)){
      if(filled%256==0)rotation=phase(-2*Pi*offset*double(filled)/16000000);
      ring_[filled%256]=(signedIq(raw[first+filled])-mean)*rotation;
      rotation=rotation*step;++filled;
    }
    int begin=std::max(0,(center-500+int(up)-1)/int(up));
    Complex sum{0,0};
    for(int j=begin;j<=last&&j<int(n);++j){
      int tap=center-j*int(up);
      if(tap>=0&&tap<=500)sum=sum+ring_[unsigned(j)%256]*(Fir[tap]*up);
    }
    if(narrow)storeComplex(m*2,sum);
    else{
      if(std::fabs(sum.r)>2047||std::fabs(sum.i)>2047)return false;
      uint16_t r=uint16_t(int16_t(std::lround(sum.r*16))),q=uint16_t(int16_t(std::lround(sum.i*16)));
      storeWord(m,uint32_t(r)|(uint32_t(q)<<16));
    }
  }
  if(narrow)narrowCount_=outputs;else timeCount_=outputs;
  return true;
}

Sync RfDecoder::acquire(const uint32_t* raw,size_t count,double offset,unsigned pci,
                         float coarse,bool sweep) {
  Sync result;
  pssPeak=sssPeak=0;
  rawStable=true;
  if(!raw||count<12000||count>16380)return result;
  uint32_t before=fingerprint(raw,count);
  if(pci>503||!resample(raw,count,offset,true))return result;
  Complex sequence[62],base[128],templ[128];
  float sss0[62],sss5[62];
  pss(pci%3,sequence);sss(pci,0,sss0);sss(pci,5,sss5);
  std::fill(base,base+128,Complex{0,0});
  for(unsigned j=0;j<62;++j)base[syncBin(j)]=sequence[j];
  fft(base,128,true);
  const int tries=sweep?13:1;
  for(int hypothesis=0;hypothesis<tries;++hypothesis){
    if(stopping())return {};
    float cfo=sweep?float(-75000+12500*hypothesis):coarse;
    for(unsigned j=0;j<128;++j)templ[j]=base[j]*phase(2*Pi*cfo*j/1920000);
    float templateEnergy=0;
    for(auto v:templ)templateEnergy+=v.power();
    float energy=0;
    for(unsigned j=0;j<128;++j)energy+=narrow(j).power();
    for(size_t start=0;start+128<=narrowCount_;++start){
      if(start){energy+=narrow(start+127).power()-narrow(start-1).power();}
      if(start<161||start+152>=narrowCount_)continue;
      Complex correlation{0,0};
      for(unsigned j=0;j<128;++j)correlation=correlation+narrow(start+j)*templ[j].conjugate();
      float score=correlation.power()/std::max(templateEnergy*energy,1e-20f);
      pssPeak=std::max(pssPeak,score);
      if(score<.15f)continue;
      Complex hp[62],equalized[62];
      for(unsigned j=0;j<128;++j)fft_[j]=narrow(start+j)*phase(-2*Pi*cfo*(start+j)/1920000);
      fft(fft_,128);
      for(unsigned j=0;j<62;++j)hp[j]=fft_[syncBin(j)]*sequence[j].conjugate();
      for(unsigned j=0;j<128;++j)fft_[j]=narrow(start-137+j)*phase(-2*Pi*cfo*(start-137+j)/1920000);
      fft(fft_,128);
      float norm=0;Complex correlations[2]={{0,0},{0,0}};
      for(unsigned j=0;j<62;++j){
        equalized[j]=fft_[syncBin(j)]*hp[j].conjugate();norm+=equalized[j].power();
        correlations[0]=correlations[0]+equalized[j]*sss0[j];
        correlations[1]=correlations[1]+equalized[j]*sss5[j];
      }
      unsigned half=correlations[1].power()>correlations[0].power()?1:0;
      float secondary=correlations[half].power()/std::max(62*norm,1e-20f);
      sssPeak=std::max(sssPeak,secondary);
      if(secondary<.20f||score*secondary<=result.pssScore*result.sssScore)continue;
      Complex cross{0,0};
      const float* ss=half?sss5:sss0;
      for(unsigned j=0;j<62;++j)cross=cross+hp[j].conjugate()*fft_[syncBin(j)]*ss[j];
      float residual=float(-std::atan2(cross.i,cross.r)/(2*Pi*137/1920000));
      result.found=true;result.pci=uint16_t(pci);result.pssStart=uint16_t(start);
      result.subframe=half?5:0;result.pssScore=score;result.sssScore=secondary;
      result.carrierCorrectionHz=cfo+residual;
      result.frameOffsetUs=.25+(double(start)-832)/1.92-result.subframe*1000.02459;
    }
    if(service)service();
  }
  rawStable=fingerprint(raw,count)==before;
  if(!rawStable)result.found=false;
  return result;
}

Sync RfDecoder::discover(const uint32_t* raw,size_t count,double offset,
                         const uint16_t* excluded,size_t excludedCount) {
  Sync result;
  pssPeak=sssPeak=0;rawStable=true;
  if(!raw||count<12000||count>16380||excludedCount>504||(!excluded&&excludedCount))return result;
  const uint32_t before=fingerprint(raw,count);
  if(!resample(raw,count,offset,true))return result;
  // Preserve narrow samples for MIB decoding. Cache the wide correlation FFT
  // in unused sample-arena words, with integer-only access to its IRAM bank.
  constexpr unsigned N=2048, Cache=4096;
  for(unsigned j=0;j<N;++j)fft_[j]=j<narrowCount_?narrow(j):Complex{0,0};
  fft(fft_,N);
  for(unsigned j=0;j<N;++j)storeComplex(Cache+2*j,fft_[j]);
  float energy=0;
  for(unsigned j=0;j<128;++j)energy+=narrow(j).power();
  for(size_t start=0;start+128<=narrowCount_;++start){
    if(start)energy+=narrow(start+127).power()-narrow(start-1).power();
    powers_[start]=std::max(energy,1e-20f);
  }
  Complex sequence[62],base[128],templ[128],hp[62],secondaryData[62];
  float sssSequence[62];
  for(unsigned root=0;root<3;++root){
    pss(root,sequence);std::fill(base,base+128,Complex{0,0});
    for(unsigned j=0;j<62;++j)base[syncBin(j)]=sequence[j];
    fft(base,128,true);
    for(int hypothesis=0;hypothesis<13;++hypothesis){
      if(stopping())return {};
      const float cfo=float(-75000+12500*hypothesis);
      float templateEnergy=0;
      std::fill(fft_,fft_+N,Complex{0,0});
      for(unsigned j=0;j<128;++j){
        templ[j]=base[j]*phase(2*Pi*cfo*j/1920000);
        templateEnergy+=templ[j].power();fft_[j]=templ[j];
      }
      fft(fft_,N);
      for(unsigned j=0;j<N;++j)fft_[j]=loadComplex(Cache+2*j)*fft_[j].conjugate();
      fft(fft_,N,true);
      // Correlation output is overwritten by the candidate's 128-point FFTs.
      // Keep only bounded strongest peaks, then verify them with the direct
      // matched filter and an independent SSS correlation.
      struct Peak { unsigned start; float score; } peaks[4]={};
      for(unsigned start=161;start+152<narrowCount_;++start){
        const float score=fft_[start].power()/(templateEnergy*powers_[start]);
        pssPeak=std::max(pssPeak,score);
        if(score<.15f)continue;
        for(unsigned k=0;k<4;++k)if(score>peaks[k].score){
          for(unsigned m=3;m>k;--m)peaks[m]=peaks[m-1];
          peaks[k]={start,score};break;
        }
      }
      for(const auto& peak:peaks){
        if(!peak.score)continue;
        const unsigned start=peak.start;
        Complex direct{0,0};
        for(unsigned j=0;j<128;++j)direct=direct+narrow(start+j)*templ[j].conjugate();
        const float score=direct.power()/(templateEnergy*powers_[start]);
        if(score<.15f)continue;
        for(unsigned j=0;j<128;++j)fft_[j]=narrow(start+j)*phase(-2*Pi*cfo*(start+j)/1920000);
        fft(fft_,128);
        for(unsigned j=0;j<62;++j)hp[j]=fft_[syncBin(j)]*sequence[j].conjugate();
        for(unsigned j=0;j<128;++j)fft_[j]=narrow(start-137+j)*phase(-2*Pi*cfo*(start-137+j)/1920000);
        fft(fft_,128);
        float norm=0;
        for(unsigned j=0;j<62;++j){
          secondaryData[j]=fft_[syncBin(j)];norm+=(secondaryData[j]*hp[j].conjugate()).power();
        }
        for(unsigned group=0;group<168;++group){
          if(group%32==0&&stopping())return {};
          const unsigned pci=3*group+root;
          bool skip=false;
          for(size_t k=0;k<excludedCount;++k)if(excluded[k]==pci){skip=true;break;}
          if(skip)continue;
          for(unsigned half=0;half<2;++half){
            sss(pci,half?5:0,sssSequence);
            Complex correlation{0,0};
            for(unsigned j=0;j<62;++j)correlation=correlation+secondaryData[j]*hp[j].conjugate()*sssSequence[j];
            const float secondary=correlation.power()/std::max(62*norm,1e-20f);
            sssPeak=std::max(sssPeak,secondary);
            if(secondary<.20f||score*secondary<=result.pssScore*result.sssScore)continue;
            Complex cross{0,0};
            for(unsigned j=0;j<62;++j)cross=cross+hp[j].conjugate()*secondaryData[j]*sssSequence[j];
            const float residual=float(-std::atan2(cross.i,cross.r)/(2*Pi*137/1920000));
            result.found=true;result.pci=uint16_t(pci);result.pssStart=uint16_t(start);
            result.subframe=half?5:0;result.pssScore=score;result.sssScore=secondary;
            result.carrierCorrectionHz=cfo+residual;
            result.frameOffsetUs=.25+(double(start)-832)/1.92-result.subframe*1000.02459;
          }
        }
      }
      if(service)service();
    }
  }
  rawStable=fingerprint(raw,count)==before;
  if(!rawStable)result.found=false;
  return result;
}

Mib RfDecoder::readMib(const Sync& sync) {
  Mib mib;
  if(!sync.found||sync.subframe!=0||sync.pssStart<137||sync.pssStart+677>narrowCount_)return mib;
  const unsigned pci=sync.pci;
  for(unsigned sym=0;sym<4;++sym){
    unsigned first=sync.pssStart+138+137*sym;
    for(unsigned j=0;j<128;++j)fft_[j]=narrow(first+j)*phase(-2*Pi*sync.carrierCorrectionHz*(first+j)/1920000);
    fft(fft_,128);
    for(unsigned j=0;j<72;++j)pilots_[0][sym][j]=fft_[j<36?92+j:j-35];
  }
  for(unsigned port=0;port<4;++port){
    unsigned l=port<2?0:1,v[4]={0,3,3,0};
    unsigned shift=(v[port]+pci)%6;
    Gold gold(1024*(14+l+1)*(2*pci+1)+2*pci+1);
    for(unsigned j=0;j<104;++j){gold.bit();gold.bit();}
    Complex h[12];
    for(unsigned j=0;j<12;++j){Complex r{gold.sign()/Sqrt2,gold.sign()/Sqrt2};h[j]=pilots_[0][l][shift+6*j]*r.conjugate();}
    for(unsigned sc=0;sc<72;++sc){
      float pos=(float(sc)-shift)/6;
      unsigned low=pos<0?0:unsigned(pos);
      pilots_[1][port][sc]=pos<=0?h[0]:(low>=11?h[11]:interpolate(h[low],h[low+1],pos-low));
    }
  }
  uint16_t scs[240];uint8_t syms[240];unsigned used=0;
  for(unsigned sym=0;sym<4;++sym)for(unsigned sc=0;sc<72;++sc){
    if(sym<2&&sc%3==pci%3)continue;
    scs[used]=uint16_t(sc);syms[used++]=uint8_t(sym);
  }
  if(used!=240)return mib;
  const unsigned portOptions[3]={4,2,1};
  for(unsigned ports:portOptions){
    for(unsigned k=0;k<240;k+=2){
      Complex a=pilots_[0][syms[k]][scs[k]],b=pilots_[0][syms[k+1]][scs[k+1]];
      if(ports==1){
        for(unsigned j=0;j<2;++j){Complex h=pilots_[1][0][scs[k+j]];powers_[k+j]=h.power();
          fft_[k+j]=pilots_[0][syms[k+j]][scs[k+j]]*h.conjugate()/(h.power()+1e-12f);}
      }else{
        unsigned p=ports==2?0:(k%4==0?0:1),q=ports==2?1:(k%4==0?2:3);
        Complex h0=(pilots_[1][p][scs[k]]+pilots_[1][p][scs[k+1]])*.5f;
        Complex h1=(pilots_[1][q][scs[k]]+pilots_[1][q][scs[k+1]])*.5f;
        float power=h0.power()+h1.power();powers_[k]=powers_[k+1]=power;
        fft_[k]=(h0.conjugate()*a+h1*b.conjugate())/(power+1e-12f);
        fft_[k+1]=(h0.conjugate()*b-h1*a.conjugate())/(power+1e-12f);
      }
    }
    std::copy(powers_,powers_+240,powers_+240);float scale=median(powers_,240);
    for(unsigned weighting=0;weighting<2;++weighting)for(unsigned ph=0;ph<4;++ph){
      if(stopping())return {};
      Gold scramble(pci);for(unsigned j=0;j<ph*480;++j)scramble.bit();
      for(unsigned k=0;k<240;++k){
        float factor=weighting?1:std::min(powers_[240+k]/scale,10.f);
        control_[2*k]=std::max(-20.f,std::min(20.f,fft_[k].r*factor))*scramble.sign();
        control_[2*k+1]=std::max(-20.f,std::min(20.f,fft_[k].i*factor))*scramble.sign();
      }
      uint8_t bits[24];uint16_t mask=ports==4?0x5555:(ports==2?0xffff:0);
      if(decodeSiControl(control_,480,24,bits,sizeof(bits),service,mask,true)!=Result::Ok)continue;
      unsigned bandwidth=readBits(bits,0,3);
      if(bandwidth>5||readBits(bits,14,10))continue;
      const unsigned rb[6]={6,15,25,50,75,100};const float ng[4]={1.f/6,.5f,1,2};
      mib.valid=true;mib.resourceBlocks=uint16_t(rb[bandwidth]);mib.ports=uint8_t(ports);
      mib.sfn=uint16_t(4*readBits(bits,6,8)+ph);mib.extendedPhich=bits[3];
      mib.phichResource=ng[readBits(bits,4,2)];return mib;
    }
  }
  return mib;
}

bool RfDecoder::grid(int start,float cfo) {
  int first=start+80;
  if(first<0||start+15360>int(timeCount_))return false;
  // Shift the first FFT sample to index zero, then run FFTs backwards. Each
  // expanded float grid row overwrites only samples already consumed.
  for(size_t j=0;j<timeCount_-first;++j)storeWord(j,loadWord(first+j));
  for(int sym=13;sym>=0;--sym){
    unsigned pos=unsigned(sym)*1096+(sym>=7?8:0);
    Complex rotation=phase(-2*Pi*cfo*(pos+first)/15360000),step=phase(-2*Pi*cfo/15360000);
    for(unsigned j=0;j<1024;++j){
      Complex x=timeSample(pos+j);
      fft_[j]=x*rotation;rotation=rotation*step;
    }
    fft(fft_,1024);
    for(unsigned j=0;j<600;++j){
      int bin=j<300?int(j)-300:int(j)-299;
      Complex value=std::fabs(bin*15000.0+carrierOffset_)>7200000?Complex{0,0}:fft_[j<300?724+j:j-299];
      storeComplex((sym*600+j)*2,value);
    }
    if(service)service();
  }
  return true;
}

void RfDecoder::estimate() {
  for(unsigned port=0;port<4;++port){
    unsigned observations=port<2?4:2;
    const unsigned times[4]={0,4,7,11};
    for(unsigned obs=0;obs<observations;++obs){
      unsigned sym=port<2?times[obs]:1+7*obs,l=sym%7,ns=2*subframe_+sym/7;
      unsigned v=port<2?(3*(l==4)+3*port)%6:(3*(ns%2)+3*(port-2))%6;
      unsigned first=(v+pci_)%6;
      Gold gold(1024*(7*(ns+1)+l+1)*(2*pci_+1)+2*pci_+1);
      for(unsigned j=0;j<60;++j){gold.bit();gold.bit();}
      for(unsigned j=0;j<100;++j){
        Complex r{gold.sign()/Sqrt2,gold.sign()/Sqrt2};
        pilots_[port][obs][j]=gridValue(sym,first+6*j)*r.conjugate();
      }
    }
  }
}

Complex RfDecoder::channel(unsigned port,unsigned sym,unsigned sc) const {
  const unsigned times[4]={0,4,7,11};
  unsigned n=port<2?4:2,low=0;
  auto t=[&](unsigned obs){return port<2?times[obs]:1+7*obs;};
  while(low+1<n&&t(low+1)<=sym)++low;
  unsigned high=low+1<n?low+1:low;
  if(sym<t(0))low=high=0;
  auto frequency=[&](unsigned obs){
    unsigned s=t(obs),l=s%7,ns=2*subframe_+s/7;
    unsigned v=port<2?(3*(l==4)+3*port)%6:(3*(ns%2)+3*(port-2))%6;
    unsigned first=(v+pci_)%6;
    float pos=(float(sc)-first)/6;
    unsigned i=pos<0?0:unsigned(pos);
    return pos<=0?pilots_[port][obs][0]:(i>=99?pilots_[port][obs][99]:
      interpolate(pilots_[port][obs][i],pilots_[port][obs][i+1],pos-i));
  };
  return low==high?frequency(low):interpolate(frequency(low),frequency(high),float(sym-t(low))/float(t(high)-t(low)));
}

void RfDecoder::equalize(const Complex* data,const uint16_t* sc,const uint8_t* sym,
                         unsigned count,unsigned offset,Complex* out,float* power) const {
  for(unsigned k=0;k<count;k+=2){
    unsigned a=(offset+k)%4==0?0:1,b=(offset+k)%4==0?2:3;
    Complex h00=channel(a,sym[k],sc[k]),h01=channel(a,sym[k+1],sc[k+1]);
    Complex h10=channel(b,sym[k],sc[k]),h11=channel(b,sym[k+1],sc[k+1]);
    power[k]=h00.power()+h11.power();power[k+1]=h01.power()+h10.power();
    out[k]=(h00.conjugate()*data[k]+h11*data[k+1].conjugate())*(Sqrt2/(power[k]+1e-12f));
    out[k+1]=(h01.conjugate()*data[k+1]-h10*data[k].conjugate())*(Sqrt2/(power[k+1]+1e-12f));
  }
}

unsigned RfDecoder::controlRegion(unsigned cfi) {
  struct Reg{uint16_t start;uint8_t sym,size;};Reg regs[400];
  bool excluded[100]={};unsigned available[100],count=0,n0=0;
  for(unsigned i=0;i<4;++i)excluded[(i*50/2+pci_)%100]=true;
  for(unsigned i=0;i<100;++i)if(!excluded[i])available[n0++]=i;
  for(unsigned j=0;j<7;++j)for(unsigned i=0;i<3;++i)excluded[available[(pci_+j+i*n0/3)%n0]]=true;
  for(unsigned sym=0;sym<cfi;++sym){
    unsigned size=sym<2?6:4;
    for(unsigned start=0;start<600;start+=size){
      if(sym==0&&excluded[start/6])continue;
      regs[count++]={uint16_t(start),uint8_t(sym),uint8_t(size)};
    }
  }
  std::sort(regs,regs+count,[&](const Reg&a,const Reg&b){
    unsigned x=a.start+(a.size==6&&unsigned(a.start)%3==pci_%3),y=b.start+(b.size==6&&unsigned(b.start)%3==pci_%3);
    return x==y?a.sym<b.sym:x<y;
  });
  const unsigned perm[32]={1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31,0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30};
  unsigned rows=(count+31)/32,pad=rows*32-count,used=0;uint16_t rank[400];
  for(unsigned col=0;col<32;++col)for(unsigned row=0;row<rows;++row){
    unsigned source=row*32+perm[col];if(source>=pad)rank[source-pad]=uint16_t(used++);
  }
  unsigned cces=std::min(count/9,16u),bits=cces*72;
  Gold gold(subframe_*512+pci_);
  for(unsigned j=0;j<cces*9;++j){
    unsigned index=(rank[j]+count-pci_%count)%count;const auto& r=regs[index];
    Complex data[4],out[4];float power[4],sorted[4];uint16_t sc[4];uint8_t sym[4];unsigned n=0;
    for(unsigned k=r.start;k<unsigned(r.start)+r.size;++k){
      if(r.size==6&&k%3==pci_%3)continue;
      sc[n]=uint16_t(k);sym[n]=r.sym;data[n]=gridValue(r.sym,k);++n;
    }
    if(n!=4)return 0;
    equalize(data,sc,sym,4,0,out,power);std::copy(power,power+4,sorted);float scale=median(sorted,4);
    for(unsigned k=0;k<4;++k){
      float factor=std::min(power[k]/scale,10.f);
      control_[8*j+2*k]=out[k].r*factor*gold.sign();
      control_[8*j+2*k+1]=out[k].i*factor*gold.sign();
    }
  }
  return bits;
}

Result RfDecoder::sharedChannel(unsigned first,unsigned rbs,unsigned cfi,unsigned rv,unsigned tbs,uint8_t* payload) {
  unsigned total=0,used=0;Complex data[4],out[4];uint16_t sc[4];uint8_t syms[4];float power[4];
  unsigned expected=0;
  for(unsigned sym=cfi;sym<14;++sym)for(unsigned k=first*12;k<(first+rbs)*12;++k)
    if(dataRe(sym,k,pci_,subframe_,cfi,first,rbs))++expected;
  unsigned stride=std::max(1u,(expected+PowerCapacity-1)/PowerCapacity),powerCount=0;
  // A uniform sample of channel powers keeps the reliability median bounded
  // without reserving a full payload-sized float array. No demodulated bits
  // or per-resource-element channel grids are stored.
  for(unsigned sym=cfi;sym<14;++sym)for(unsigned k=first*12;k<(first+rbs)*12;++k){
    if(!dataRe(sym,k,pci_,subframe_,cfi,first,rbs))continue;
    sc[used]=uint16_t(k);syms[used]=uint8_t(sym);data[used++]=gridValue(sym,k);
    if(used==4){
      equalize(data,sc,syms,4,total,out,power);
      if((total/4)%stride==0&&powerCount+4<=PowerCapacity){std::copy(power,power+4,powers_+powerCount);powerCount+=4;}
      total+=4;used=0;
    }
  }
  if(used||!total)return Result::Unsupported;
  float scale=median(powers_,powerCount);
  Result status=turbo_->begin(tbs,rv);if(status!=Result::Ok)return status;
  Gold gold(65535u*16384+subframe_*512+pci_);total=0;used=0;
  for(unsigned sym=cfi;sym<14;++sym){
    if(service)service();
    for(unsigned k=first*12;k<(first+rbs)*12;++k){
      if(!dataRe(sym,k,pci_,subframe_,cfi,first,rbs))continue;
      sc[used]=uint16_t(k);syms[used]=uint8_t(sym);data[used++]=gridValue(sym,k);
      if(used==4){
        equalize(data,sc,syms,4,total,out,power);
        for(unsigned j=0;j<4;++j){
          float factor=4*std::min(power[j]/scale,10.f);
          status=turbo_->push(std::max(-30.f,std::min(30.f,out[j].r*factor))*gold.sign());
          if(status!=Result::Ok)return status;
          status=turbo_->push(std::max(-30.f,std::min(30.f,out[j].i*factor))*gold.sign());
          if(status!=Result::Ok)return status;
        }
        total+=4;used=0;
      }
    }
  }
  return turbo_->finish(payload,MaxTransportBits/8,8,service);
}

Result RfDecoder::readSystem(const uint32_t* raw,size_t count,double offset,unsigned pci,unsigned subframe,
                              uint8_t* payload,size_t& bytes) {
  bytes=0;grants=0;transportBits=0;
  if(!payload||pci>503||subframe>9)return Result::InvalidInput;
  pci_=pci;subframe_=subframe;carrierOffset_=offset;
  if(!resample(raw,count,offset,false))return Result::InvalidInput;
  struct Timing{int start;float score,cfo;};Timing candidates[121];unsigned nc=0;
  for(int start=-160;start<=320;start+=4){
    int pos=start;Complex cross{0,0};float ea=0,eb=0;
    for(unsigned sym=0;sym<14;++sym){
      int cp=sym%7==0?80:72;
      int first=std::max(pos+16,32),last=std::min(pos+cp-8,int(timeCount_)-1024-32);
      for(int j=first;j<last;++j){
        Complex a=timeSample(j),b=timeSample(j+1024);
        cross=cross+a.conjugate()*b;ea+=a.power();eb+=b.power();
      }
      pos+=cp+1024;
    }
    candidates[nc++]={start,cross.power()/std::max(ea*eb,1e-20f),float(std::atan2(cross.i,cross.r)*15360000/(2*Pi*1024))};
  }
  std::sort(candidates,candidates+nc,[](const Timing&a,const Timing&b){return a.score>b.score;});
  Timing selected[3];unsigned ns=0;
  for(unsigned i=0;i<nc&&ns<3;++i){
    bool distinct=true;for(unsigned j=0;j<ns;++j)if(std::abs(selected[j].start-candidates[i].start)<24)distinct=false;
    if(distinct)selected[ns++]=candidates[i];
  }
  for(unsigned t=0;t<ns;++t)for(unsigned mode=0;mode<2;++mode){
    if(stopping())return Result::CrcFailed;
    float cfo=mode?0:selected[t].cfo;
    for(unsigned refinement=0;refinement<2;++refinement){
      if(!resample(raw,count,offset,false)||!grid(selected[t].start,cfo))continue;
      estimate();
      if(refinement==0){
        Complex cross{0,0};float ea=0,eb=0;
        for(unsigned port=0;port<2;++port)for(unsigned obs=0;obs<2;++obs)for(unsigned j=0;j<100;++j){
          Complex a=pilots_[port][obs][j],b=pilots_[port][obs+2][j];cross=cross+a.conjugate()*b;ea+=a.power();eb+=b.power();
        }
        float coherence=std::sqrt(cross.power()/std::max(ea*eb,1e-20f));
        if(coherence>.1f){
          float fine=float(std::atan2(cross.i,cross.r)/(2*Pi*.0005));
          if(!resample(raw,count,offset,false)||!grid(selected[t].start,cfo+fine))continue;
          estimate();
        }
      }
      timingStart=selected[t].start;cpScore=selected[t].score;appliedCfoHz=cfo;
      for(unsigned cfi=1;cfi<=3;++cfi){
        if(stopping())return Result::CrcFailed;
        unsigned length=controlRegion(cfi),cces=length/72;
        for(unsigned aggregation=4;aggregation<=8;aggregation+=4)
          for(unsigned firstCce=0;firstCce+aggregation<=cces;firstCce+=aggregation){
            if(stopping())return Result::CrcFailed;
            uint8_t bits[27];
            if(decodeSiControl(control_+firstCce*72,aggregation*72,27,bits,sizeof(bits),service,0xffff,true)!=Result::Ok)continue;
            if(bits[0]!=1||bits[1]!=0)continue;
            unsigned riv=readBits(bits,2,11),rbs=riv/50+1,first=riv%50;
            if(first+rbs>50){rbs=51-riv/50;first=49-riv%50;}
            unsigned index=readBits(bits,13,5),rv=readBits(bits,22,2),tpc=readBits(bits,24,2);
            const unsigned tbs2[14]={32,56,72,104,120,144,176,224,256,296,328,376,440,488};
            const unsigned tbs3[10]={56,88,144,176,208,224,256,328,392,456};
            if(!rbs||first+rbs>50||index>=(tpc&1?10u:14u))continue;
            unsigned size=(tpc&1)?tbs3[index]:tbs2[index];++grants;
            Result status=sharedChannel(first,rbs,cfi,rv,size,payload);
            if(status==Result::Ok){bytes=size/8;transportBits=size;return status;}
          }
      }
    }
  }
  return Result::CrcFailed;
}
} // namespace esp32_lte_time
