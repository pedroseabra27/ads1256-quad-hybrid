## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Aggregate SPS | 120,000 | 4 * 30 kSPS |
| Latency (sample avail) | <1 ms | From hardware to Python |
| Sync accuracy | <50 µs | Inter-device skew |
| CPU usage | <15% | On 4-core baseline |
| Memory | <100 MB | All buffers + code |

### Optimization Techniques (planned / partial)
* Real-time scheduling (SCHED_FIFO) for acquisition threads.
* CPU affinity pinning and NUMA locality.
* Lock-free single-producer multi-consumer ring buffer.
* Batch SPI transfers & DMA (future).
* Zero-copy export to NumPy (memoryview + ndarray wrapper).
* Preallocated buffers & pooling.
* Frame period jitter tracking (EMA, RMS, max) for timing diagnostics.

### Timing & Jitter Metrics
The system maintains:
* avg_frame_period_ns: EMA of frame-to-frame period.
* rms_frame_jitter_ns: Exponential RMS of (period - avg).
* max_frame_jitter_ns: Maximum absolute deviation since start.
* jitter_hist: Distribution buckets of absolute jitter (ns):
	* [0] <1k
	* [1] <2k
	* [2] <5k
	* [3] <10k
	* [4] <20k
	* [5] <50k
	* [6] <100k
	* [7] >=100k

Usage (Python):
```python
m = system.get_metrics()
print(m['avg_frame_period_ns'], m['rms_frame_jitter_ns'], m['max_frame_jitter_ns'])
```
Interpretation:
* Low RMS vs average indicates stable pacing.
* Large max but small RMS implies rare outlier (investigate GC, scheduler, IRQ delays).
* If RMS grows with load, consider enabling edge-driven DRDY (libgpiod) and real-time thread priority.

### Logging
Built-in lightweight logging helps correlating timing anomalies. Levels:
* 10 DEBUG
* 20 INFO (default)
* 30 WARN
* 40 ERROR

Python usage:
```python
from ads1256_quad_api import _core
_core.set_log_level(10)  # enable debug
print(_core.get_log_level())
```
Native logs go to stderr; can be disabled at compile time with `-DADS1256_DISABLE_LOGGING`.

### Benchmark Scripts
`examples/performance_benchmark.py` (placeholder) will exercise throughput, latency, sustained runs.
