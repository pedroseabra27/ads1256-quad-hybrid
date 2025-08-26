# ADS1256 Quad Hybrid System

High-performance hybrid C / Cython / Python framework for managing 4 ADS1256 24‑bit ADC devices across 2 SPI buses (SPI0 & SPI1) with synchronization, real-time options and performance monitoring.

Este repositório é um esqueleto inicial (skeleton) baseado na documentação fornecida. Você pode expandir gradualmente até a implementação completa.

## Summary / Resumo

| Feature | Status |
|---------|--------|
| 4 devices (ADS1256) | Skeleton (fake data) |
| Dual SPI (SPI0/SPI1) | Config structure only |
| Hardware sync (GPIO) | Stub (libgpiod planned) |
| DMA | Not implemented |
| Real-time (SCHED_FIFO) | Placeholder configuration |
| Zero-copy NumPy | Partial (direct buffer copy) |
| Benchmarks | Basic throughput stub |
| Export (csv/hdf5/mat) | Implemented (mat via scipy) |

Badge (CI): ![CI](https://github.com/pedroseabra27/ads1256-quad-hybrid/actions/workflows/ci.yml/badge.svg)

## Installation (Development)

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
```

## Quick Test

```bash
python examples/basic_quad_test.py
```

## Example (Python)

```python
from ads1256_quad_api import create_default_system
system = create_default_system()
system.initialize()
with system.acquisition_context():
    data = system.read_voltages_numpy(1000)
    print(data.shape)
    print(system.get_stats())
```

## Project Layout

```
src/
  ads1256_core/      # C core (SPI, buffering, sync stubs)
  ads1256_quad_api/  # Cython + Python high-level API
examples/            # Usage examples
scripts/             # CLI tools & services
.github/             # CI, issue templates
```

## Roadmap (Short)

1. Implement real SPI read sequence (reset, register config, continuous read)
2. Add acquisition thread + ring buffer real
3. Implement GPIO sync (libgpiod) pulses & timestamping
4. Add calibration and voltage conversion based on Vref & gain
5. Performance optimization (affinity, hugepages optional, lock-free queue)

## Contributing

See CONTRIBUTING.md. Pull requests welcome.

## License

GPL-2.0 (see LICENSE).

---

## (PT) Notas Iniciais
- Dados atuais são sintéticos (rampa) gerados no C.
- Substituir por implementação de acesso ao /dev/spidevX.Y.
- Para Raspberry Pi 5 habilite SPI0/SPI1 no config.txt (dtparam=spi=on, overlays de SPI1).

## (EN) Initial Notes
- Current data are synthetic ramp samples produced in C as a placeholder.
- Replace with real ADS1256 register init + read loop.
