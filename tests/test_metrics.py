import time
import math
import pytest

from ads1256_quad_api import _core

def test_metrics_fields_presence():
    sys = _core.CoreSystem().create_default()
    sys.start()
    # Pull some frames (synthetic for now)
    for _ in range(10):
        f = sys.pop_frame()
        if f is None:
            time.sleep(0.001)
    m = sys.get_metrics()
    # Required keys
    expected = {
        'frames_produced', 'dropped_frames', 'avg_frame_period_ns',
        'rms_frame_jitter_ns', 'max_frame_jitter_ns',
        'last_frame_acq_ns', 'avg_frame_acq_ns',
        'drdy_waits', 'drdy_timeouts', 'last_drdy_wait_ns', 'avg_drdy_wait_ns', 'throughput_fps'
    }
    missing = expected - set(m.keys())
    assert not missing, f"Missing metric keys: {missing}"
    assert m['frames_produced'] >= 0
    assert m['avg_frame_period_ns'] >= 0
    assert m['rms_frame_jitter_ns'] >= 0
    assert m['max_frame_jitter_ns'] >= 0
    assert 'jitter_hist' in m and len(m['jitter_hist']) == 8
    sys.stop()

@pytest.mark.parametrize('loops', [50])
def test_jitter_values_non_negative(loops):
    sys = _core.CoreSystem().create_default()
    sys.start()
    for _ in range(loops):
        sys.pop_frame()
        time.sleep(0.0005)
    m = sys.get_metrics()
    assert m['rms_frame_jitter_ns'] >= 0
    assert m['max_frame_jitter_ns'] >= 0
    sys.stop()
