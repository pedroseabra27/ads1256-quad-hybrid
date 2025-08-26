## Testing & Benchmarking

### Layers
1. Unit (C): error translation, config validation (future with CTest or Unity).
2. Unit (Python): object construction, default settings.
3. Integration (simulated): synthetic data path end-to-end.
4. Performance: throughput + latency (requires hardware or simulator).
5. Synchronization: quality metrics under induced skew.

### Quick Commands
```
pytest -q
python examples/basic_quad_test.py
```

### Simulated Data
Current implementation returns ramp patterns until real SPI logic added.
