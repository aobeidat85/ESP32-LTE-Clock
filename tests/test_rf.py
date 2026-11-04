"""Raw RF replay tests. No newly received signals or host UTC inputs."""
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from test_decoder import native, run, FIXTURES, PROOF
from lte_sync_sequences import pss, sss, symbol
from scipy.signal import resample_poly


@pytest.mark.parametrize('window',[1,2,5,10,15,20,40])
def test_entire_advertised_si_window_is_searched(native,window):
    offsets=run(native,'siwindow',window)
    period=2*window-1
    assert sorted(set(offsets[:period]))==list(range(window))
    assert offsets[:period]==offsets[period:]
    assert offsets[:period:2]==[0]*window
    if window==20:assert offsets[:8]==[0,10,0,1,0,11,0,2]
    if window==40:assert offsets[:6]==[0,10,0,20,0,30]


def test_invalid_si_windows_are_rejected(native):
    for window in [0,41,65535]:assert run(native,'siwindow',window)==[-1]


def test_scan_plan_covers_every_reachable_raster_point_once(native):
    plan=run(native,'scanplan')
    expected=set()
    for first,last in [(21100,21999),(19300,19949),(18420,18799),(23500,23599),(26200,26899)]:
        expected.update(range(first,last+1))
    assert len(plan)==len(expected)==len({f for f,lo in plan})
    assert {f for f,lo in plan}==expected
    assert [f for f,lo in plan[:3]]==[21550,21500,21600]
    assert all(1842<=lo<=2800 and abs(f*100000-lo*1000000)<=6000000 for f,lo in plan)


def test_blind_discovery_decodes_recorded_mib_without_pci(native):
    r=run(native,'discover',FIXTURES/'mib-raw.u32',1000000,65535,0)
    assert r['found'] and r['pci']==284 and r['mib']
    assert (r['sfn'],r['nrb'],r['ports'],r['ng'])==(49,50,4,1)
    assert abs(r['start']-853)<=1


@pytest.mark.parametrize('pci,half,cfo',[(0,0,0),(1,5,-51000),(2,0,38000),
                                      (57,5,0),(283,0,-52000),(503,5,62000)])
def test_blind_discovery_all_pss_roots_and_sss_extremes(native,tmp_path,pci,half,cfo):
    # Independent NumPy frequency-domain PSS/SSS and SciPy interpolation.
    y=np.zeros(1966,complex)
    for start,sequence in [(716,sss(pci//3,pci%3,half)),(853,pss(pci%3))]:
        ofdm=symbol(sequence)
        y[start:start+128]=ofdm
        y[start-9:start]=ofdm[-9:]
    x=resample_poly(y,25,3)[:16380]
    x*=np.exp(2j*np.pi*(1000000+cfo)*np.arange(len(x))/16000000)
    x*=220/max(abs(x))
    rng=np.random.default_rng(pci+half)
    x+=rng.normal(0,1,len(x))+1j*rng.normal(0,1,len(x))
    i=np.clip(np.rint(x.real),-512,511).astype(np.int32)
    q=np.clip(np.rint(x.imag),-512,511).astype(np.int32)
    p=tmp_path/'synthetic.u32';(((i&1023)<<10)|(q&1023)).astype('<u4').tofile(p)
    r=run(native,'discover',p,1000000,65535,0)
    assert r['found'] and r['pci']==pci and r['sf']==half
    assert abs(r['cfo']-cfo)<8000
    assert not r['mib'] # A synchronization hypothesis alone cannot verify a cell.


def test_blind_discovery_exclusions_and_cancellation(native):
    p=FIXTURES/'mib-raw.u32'
    excluded=run(native,'discover',p,1000000,284,0)
    assert not excluded['found'] or excluded['pci']!=284
    assert not excluded['mib'] # SSS cross-correlation hypotheses are not CRC-verified cells.
    assert not run(native,'discover',p,1000000,65535,0,-1)['found']


def test_blind_discovery_rejects_flat_and_noise(native,tmp_path):
    rng=np.random.default_rng(584)
    for name,words in [('flat',np.zeros(16380,dtype='<u4')),
                       ('noise',rng.integers(0,1<<20,16380,dtype='<u4'))]:
        p=tmp_path/(name+'.u32');words.tofile(p)
        r=run(native,'discover',p,1000000,65535,0)
        assert not r['found'] and not r['mib']


def test_raw_iq_to_crc_valid_sib8_and_utc(native):
    p=FIXTURES/'sib8-raw.u32'
    assert hashlib.sha256(p.read_bytes()).hexdigest()==PROOF['raw_sha256']
    r=run(native,'rf',p,947864.7,284,0)
    assert r['status']==0 and r['payload']==PROOF['payload_hex']
    assert r['ticks']==PROOF['ticks'] and r['unix_us']==PROOF['unix_us']
    assert r['timing']==116 and r['grants']>=1 and r['workspace']<165000


def test_raw_sib1_schedule_is_read_instead_of_assumed(native):
    p=FIXTURES/'sib1-raw.u32'
    assert hashlib.sha256(p.read_bytes()).hexdigest()=='fa591c19666ae7aa7cd15fc721e3e9b964cce8f98a3444e499a8d711b56cbadd'
    r=run(native,'rf',p,947864.7,284,5)
    assert r['status']==0 and r['schedule_status']==0 and r['cell']==45012513
    assert (r['period'],r['window'],r['window_start'])==(64,20,20)


@pytest.mark.parametrize('offset,coarse,sweep',[(947132.8,0,0),(1000000,-50000,1)])
def test_pss_sss_and_mib_without_a_saved_frame_number(native,offset,coarse,sweep):
    info=json.loads((FIXTURES/'mib-raw.json').read_text())
    p=FIXTURES/'mib-raw.u32'
    assert hashlib.sha256(p.read_bytes()).hexdigest()==info['metadata']['sha256']
    r=run(native,'sync',p,offset,284,coarse,sweep)
    assert r['found'] and r['mib'] and r['sf']==0
    # Adjacent correlation peaks can swap under MSVC/GCC floating-point
    # rounding during the coarse frequency sweep (one 1.92 MS/s sample).
    assert abs(r['start']-853)<=1
    assert (r['sfn'],r['nrb'],r['ports'],r['ng'])==(49,50,4,1)


def test_constant_raw_samples_cannot_acquire_or_set_time(native,tmp_path):
    p=tmp_path/'flat.u32';np.zeros(16380,dtype='<u4').tofile(p)
    assert not run(native,'sync',p,1000000,284,0,0)['found']
    r=run(native,'rf',p,947864.7,284,0)
    assert r['status']!=0 and not r['payload'] and r['unix_us']==0


def test_frame_period_requires_fresh_monotonic_mibs_with_agreeing_sfn(native):
    r=run(native,'fit',1000000,1020,1200005,16)
    assert r['ok'] and r['period']==10000.25
    for a,s,b,t in [(2000000,1,1000000,2),(1000000,10,1200005,29),(1000000,10,1000000,10),
                     (1000000,1024,1200005,20),(1000000,10,1200100,30)]:
        assert not run(native,'fit',a,s,b,t)['ok']


def test_si_window_reference_is_end_of_window_not_capture_or_decode_time(native):
    r=run(native,'reference',64,20,20,898,0,542827285,116,10000.2459)
    assert r=={'status':0,'reference':542847293}
    # Second frame in the same SI window references that same window boundary.
    r=run(native,'reference',64,20,20,899,0,542837285,116,10000.2459)
    assert r=={'status':0,'reference':542847293}
    assert run(native,'reference',64,20,20,900,0,542847285,116,10000.2459)['status']!=0
