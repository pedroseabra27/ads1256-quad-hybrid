# ADS1256 Quad Hybrid System

High-performance hybrid C / Cython / Python framework for managing 4 ADS1256 24‑bit ADC devices across 2 SPI buses (SPI0 & SPI1) with synchronization, real-time options and performance monitoring.

Este repositório agora contém a estrutura inicial do core C, API Python e documentação separada em `docs/` baseada no material completo fornecido.

## Summary / Resumo

| Feature | Status |
|---------|--------|
| 4 devices (ADS1256) | Implemented (sequential + parallel per bus) |
| Dual SPI (SPI0/SPI1) | Active (bus worker threads) |
| RDATAC mode | Basic (MUX sweep with discard+read) |
| DRDY (GPIO) | Abstraction + metrics (libgpiod integration flag pending) |
| Zero-copy frame access | Implemented (`pop_frame_view`) |
| Per-channel gain scaling | Implemented |
| Multi-thread acquisition | Main + per-bus workers |
| Metrics (throughput, latency, DRDY) | Implemented (EMA + counts) |
| Trigger/Burst | Pending |
| Sync quality metric | Pending |
| Config loader | Pending |
| Logging structured | Pending |
| Export (csv/hdf5/parquet) | Pending (not implemented yet) |
| Tests / CI | Minimal (import only) |

Badge (CI): ![CI](https://github.com/pedroseabra27/ads1256-quad-hybrid/actions/workflows/ci.yml/badge.svg)

## Installation (Development)

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
# (compila extensão Cython; se falhar use: python setup.py build_ext --inplace)

# Optional: enable libgpiod DRDY edge timing (Linux/RPi)
# Install system dependency:
#   sudo apt update && sudo apt install -y libgpiod-dev gpiod
# Build with flag:
#   CFLAGS="-DADS1256_HAVE_GPIOD" pip install -e .
# Or set environment before building the extension.
```

## Quick Test

```bash
python examples/basic_quad_test.py
# Test hardware (necessita SPI devices e permissões):
python examples/hw_thread_read.py
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
docs/                    # Detailed architecture & usage docs
src/
  ads1256_core/          # C core (headers in include/, synthetic stubs)
  ads1256_quad_api/      # Python high-level API (Cython bridge TBD)
examples/                # Usage examples
tests/                   # Pytests
```

Key docs:
* `docs/ARCHITECTURE.md`
* `docs/CONFIGURATION.md`
* `docs/ACQUISITION.md`
* `docs/PERFORMANCE.md`
* `docs/TESTING.md`
* `docs/ROADMAP.md`

## Roadmap (Short)

See `docs/ROADMAP.md`.

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
