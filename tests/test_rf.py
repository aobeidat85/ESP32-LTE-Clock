"""Raw RF replay tests. No newly received signals or host UTC inputs."""
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from test_decoder import native, run, FIXTURES, PROOF


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
    assert (r['start'],r['sfn'],r['nrb'],r['ports'],r['ng'])==(853,49,50,4,1)


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
