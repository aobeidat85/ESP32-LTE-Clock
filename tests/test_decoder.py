"""Compare the actual portable C++ decoder against independent Python paths."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from pycrate_asn1dir import RRCLTE

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path[:0] = [str(ROOT / 'tools'), str(HERE / 'reference')]
from build_native import build
from vendor.estevez_lte import turbo_encode, rate_matching_turbo
from lte_time_fields import sib8_time
from lte_time_results import utc_from_table
from lte_pbch import trellis, crc16
from lte_downlink import convolutional_map

FIXTURES = ROOT / 'tests/fixtures/lte'
PROOF = json.loads((FIXTURES / 'sib8.json').read_text())
OK, INVALID, UNSUPPORTED, CRC, MALFORMED, NO_TIME, EXPIRED, LEAP, REFERENCE = range(9)


@pytest.fixture(scope='session')
def native():
    return Path(os.environ['LTE_NATIVE']) if 'LTE_NATIVE' in os.environ else build()


def run(native, *args):
    return json.loads(subprocess.check_output([str(native), *map(str, args)], text=True))


def floats(tmp_path, values):
    p = tmp_path / 'llr.f32'
    np.asarray(values, dtype='<f4').tofile(p)
    return p


def encode(blocks):
    m = RRCLTE.EUTRA_RRC_Definitions.BCCH_DL_SCH_Message
    m.set_val({'message': ('c1', ('systemInformation', {'criticalExtensions': (
        'systemInformation-r8', {'sib-TypeAndInfo': blocks})}))})
    return m.to_uper()


def block(ticks, asynchronous=False):
    return {'systemTimeInfo': {'cdma-EUTRA-Synchronisation': not asynchronous,
        'cdma-SystemTime': ('asynchronousSystemTime' if asynchronous else 'synchronousSystemTime',
                           (ticks, 49 if asynchronous else 39))}}


def test_recorded_rf_control_transport_time_and_provenance(native, tmp_path):
    for name, digest in PROOF['fixture_sha256'].items():
        assert hashlib.sha256((FIXTURES / name).read_bytes()).hexdigest() == digest
    control = run(native, 'control', FIXTURES / 'sib8-control.f32', len(PROOF['dci_bits']))
    assert control == {'status': OK, 'bits': PROOF['dci_bits']}
    llr = np.fromfile(FIXTURES / 'sib8-pdsch.i8', dtype='i1').astype(float) / PROOF['llr_scale']
    r = run(native, 'turbo', floats(tmp_path, llr), PROOF['transport_bits'], PROOF['rv'])
    assert r['transport_status'] == OK and r['payload'] == PROOF['payload_hex']
    assert r['rrc']['ticks'] == PROOF['ticks'] and r['rrc']['unix_us'] == PROOF['unix_us']


@pytest.mark.parametrize('rv', range(4))
@pytest.mark.parametrize('payload', [bytes.fromhex(PROOF['payload_hex']), bytes(range(32)), bytes(range(61))])
def test_turbo_reference_encoder_all_rv_with_erasures(native, tmp_path, rv, payload):
    encoded = np.array(turbo_encode(payload))
    # Independent Python encoder/rate matcher; C++ implements only decoding.
    bits = np.array([encoded[a, b] for a, b in rate_matching_turbo(encoded.shape[1], 1800, rv)], float)
    rng = np.random.default_rng(rv)
    llr = 4 * (1 - 2 * bits) + rng.normal(0, .4, len(bits))
    llr[::7] = 0
    r = run(native, 'turbo', floats(tmp_path, llr), len(payload) * 8, rv)
    assert r['transport_status'] == OK and r['payload'] == payload.hex()


@pytest.mark.parametrize('values', [np.zeros(1500), np.full(1500, np.nan), np.full(1500, np.inf),
    np.random.default_rng(14).normal(size=1500)])
def test_noise_missing_and_nonfinite_llrs_cannot_produce_time(native, tmp_path, values):
    r = run(native, 'turbo', floats(tmp_path, values), 440, 0)
    assert r['transport_status'] != OK and r['payload'] == ''


def test_control_crc_and_input_validation(native, tmp_path):
    good = np.fromfile(FIXTURES / 'sib8-control.f32', dtype='<f4')
    for llr in [np.zeros_like(good), np.full_like(good, np.nan),
                np.random.default_rng(12).normal(size=len(good))]:
        assert run(native, 'control', floats(tmp_path, llr), 27)['status'] != OK
    for size, rv, status in [(504, 0, UNSUPPORTED), (440, 4, INVALID), (439, 0, INVALID), (8, 0, UNSUPPORTED)]:
        assert run(native, 'turbo', floats(tmp_path, good), size, rv)['transport_status'] == status


@pytest.mark.parametrize('asynchronous', [False, True])
@pytest.mark.parametrize('seconds', [0, 60, 1477776505, 1490000000])
def test_typed_uper_time_and_utc_match_python(native, tmp_path, asynchronous, seconds):
    units = 153600 if asynchronous else 100
    ticks = seconds * units + units // 2
    b = block(ticks, asynchronous)
    p = tmp_path / 'sib8.bin'; p.write_bytes(encode([('sib8', b)]))
    r = run(native, 'rrc', p)
    expected = sib8_time(b)
    utc, _, _ = utc_from_table(expected['gps_calendar'], (ROOT / 'extras/iers-leap-seconds.dat').read_text())
    assert r['status'] == OK and r['ticks'] == ticks and r['width'] == (49 if asynchronous else 39)
    assert r['utc_status'] == OK and r['unix_us'] == int(utc.timestamp() * 1e6)


def test_no_time_duplicates_truncation_and_nonzero_trailer(native, tmp_path):
    p = tmp_path / 'bad.bin'
    p.write_bytes(encode([('sib8', {})])); assert run(native, 'rrc', p)['status'] == NO_TIME
    p.write_bytes(encode([('sib8', block(10)), ('sib8', block(20))]))
    assert run(native, 'rrc', p)['status'] != OK
    original = (FIXTURES / 'sib8-payload.bin').read_bytes()
    for n in range(1, 49):
        p.write_bytes(original[:n]); assert run(native, 'rrc', p)['status'] != OK
    p.write_bytes(original + b'\x01'); assert run(native, 'rrc', p)['status'] == MALFORMED
    # A random payload containing a recognizable counter is not a SIB8 parser input.
    p.write_bytes(b'\xff' + original[1:]); assert run(native, 'rrc', p)['status'] != OK


def test_leap_expiry_insertion_and_bit_width_validation(native):
    assert run(native, 'utc', 1 << 39, 39)['utc_status'] == INVALID
    assert run(native, 'utc', 1, 48)['utc_status'] == INVALID
    gps_epoch = 315964800
    expiry = int(datetime(2027, 6, 28, tzinfo=timezone.utc).timestamp())
    assert run(native, 'utc', (expiry - gps_epoch + 18) * 100, 39)['utc_status'] == EXPIRED
    boundary = int(datetime(2017, 1, 1, tzinfo=timezone.utc).timestamp())
    assert run(native, 'utc', (boundary - gps_epoch + 17) * 100, 39)['utc_status'] == LEAP
    assert run(native, 'utc', (boundary - gps_epoch + 18) * 100, 39)['unix_us'] == boundary * 1000000


def test_monotonic_reference_rejects_reset_future_and_stale_time(native):
    utc = PROOF['unix_us']
    assert run(native, 'advance', utc, 100, 1100, 1000) == {'status': OK, 'unix_us': utc + 1000}
    for ref, now, age in [(100, 99, 1000), (-1, 100, 1000), (100, 1101, 1000)]:
        assert run(native, 'advance', utc, ref, now, age)['status'] == REFERENCE


def test_workspace_is_bounded(native):
    size = int(subprocess.check_output([str(native), 'memory'], text=True))
    assert size <= 32 * 1024


@pytest.mark.parametrize('seed',range(8))
def test_fast_closed_path_control_matches_independent_encoder_and_exact_reader(native,tmp_path,seed):
    rng=np.random.default_rng(seed)
    payload=rng.integers(0,2,27)
    message=np.r_[payload,crc16(payload)^1]
    state=0
    for bit in message[-6:]:state=((state<<1)|int(bit))&63
    encoded=[]
    for bit in message:
        encoded.append(trellis()[state,bit]);state=((state<<1)|int(bit))&63
    encoded=np.array(encoded).T.flatten()[convolutional_map(len(message))]
    transmitted=np.resize(encoded,288 if seed%2 else 576)
    llr=4*(1-2*transmitted)+rng.normal(0,.8,len(transmitted));llr[::11]=0
    p=floats(tmp_path,llr)
    expected={'status':OK,'bits':''.join(map(str,payload.tolist()))}
    assert run(native,'control',p,27)==expected
    assert run(native,'control_fast',p,27)==expected


@pytest.mark.parametrize('values',[np.zeros(576),np.full(576,np.nan),np.full(576,np.inf),
                                 np.random.default_rng(45).normal(size=576)])
def test_fast_control_rejects_absent_invalid_and_noise_inputs(native,tmp_path,values):
    assert run(native,'control_fast',floats(tmp_path,values),27)['status']!=OK
