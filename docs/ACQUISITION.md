## Acquisition Modes

### Continuous
Starts background acquisition threads; user polls or drains ring buffers.

### Triggered
Edge / level external GPIO cause capture of N samples per device (pre/post future enhancement).

### Burst
Synchronous short capture at a specific rate for bounded sample counts.

### Real-time Processor
User supplies function(buffer_view)->result executed in dedicated thread consuming from ring with minimal latency.

### Voltage Conversion
Raw 24-bit code (signed) -> volts:

```
code_signed / (2^23 - 1) * Vref / gain
```

Where `code_signed` is left-aligned from ADS1256 output, two's complement.

### Timestamp Strategy (planned)
1. Acquire coarse monotonic clock per batch.
2. Interpolate per-sample timestamp assuming ideal spacing; adjust drift each sync pulse.
3. Provide optional high resolution hardware timestamp if kernel driver exposes it.

### Synchronization Quality Metric
`quality = 1 - (abs(period_error_mean) + jitter_rms / target_period) / 2`  (heuristic placeholder).
