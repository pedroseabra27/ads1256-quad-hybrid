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

### Benchmark Scripts
`examples/performance_benchmark.py` (placeholder) will exercise throughput, latency, sustained runs.
