## Architecture Overview

High-performance hybrid stack for managing 4 ADS1256 devices over 2 SPI buses.

Layers:

1. Python Application Layer – user scripts, analysis, export.
2. Python High-Level API (`ads1256_quad_api`) – orchestration, configuration, data handling helpers.
3. Cython Interface – thin, typed bridge exposing zero-copy buffers & structs.
4. C Core (`ads1256_core`) – real-time acquisition engine, ring buffers, synchronization, error handling.
5. Kernel / OS – spidev, gpio (libgpiod), scheduler, optional DMA drivers.
6. Hardware – 4x ADS1256 on SPI0 (2 devices) + SPI1 (2 devices) with shared / coordinated sync GPIO.

### Data Flow

```
ADS1256 -> SPI -> kernel spidev -> C core DMA/read thread -> lock-free ring -> Cython view -> NumPy array -> User code
```

### Synchronization

Hardware sync GPIO pulses (master) optionally fan-out to all devices; software corrects minor skew (<50 µs target) with timestamp interpolation.

### Threads (planned)

| Thread | Role | Notes |
|--------|------|-------|
| rt_acq (per SPI bus) | Burst / continuous reads | SCHED_FIFO, pinned core |
| sync_monitor | Measures pulse intervals | Optional |
| perf_monitor | Aggregates metrics | 1 Hz interval |
| processor (optional) | Real-time transform | User supplied function |

### Buffers

Lock-free single producer / multiple consumer ring per SPI bus. Exposed to Python as zero-copy memoryview / NumPy without extra allocations.

### Build Targets

| Target | Type | Description |
|--------|------|-------------|
| ads1256_core | Static lib | Core C implementation |
| ads1256_quad_api.core | Cython extension | Bridge layer |

---

See `CONFIGURATION.md` and `ACQUISITION.md` for more detail.
