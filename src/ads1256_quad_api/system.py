"""High-level Python API skeleton for ADS1256 Quad System.

Currently backed by synthetic C core producing ramp data.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional
import threading
import math


@dataclass
class ChannelSettings:
    enabled: bool = True
    gain: int = 1
    input_mode: str = "single"
    label: str = ""


@dataclass
class DeviceSettings:
    device_id: int = 0
    spi_bus: int = 0
    chip_select: int = 0
    sample_rate: int = 30000
    buffer_size: int = 64 * 1024
    auto_calibration: bool = True
    channels: List[ChannelSettings] = field(default_factory=lambda: [ChannelSettings(label=f"CH{i}") for i in range(8)])


@dataclass
class SystemSettings:
    devices: List[DeviceSettings] = field(default_factory=list)
    sync_mode: str = "hardware"
    use_dma: bool = True
    real_time: bool = False
    rt_priority: int = 80


def create_default_system() -> "ADS1256QuadSystem":
    devices = []
    for did in range(4):
        devices.append(DeviceSettings(device_id=did, spi_bus=did // 2, chip_select=did % 2))
    settings = SystemSettings(devices=devices)
    return ADS1256QuadSystem(settings)


class ADS1256QuadSystem:
    def __init__(self, settings: SystemSettings):
        self.settings = settings
        self._running = False
        self._lock = threading.Lock()
        self._counter = 0

    # Placeholder for integration with C core later
    def initialize(self) -> None:
        # In real impl: build C config struct, create system via C API
        pass

    def start_continuous_acquisition(self) -> None:
        with self._lock:
            self._running = True

    def stop_continuous_acquisition(self) -> None:
        with self._lock:
            self._running = False

    # Context manager for acquisition
    def acquisition_context(self):
        class _Ctx:
            def __init__(self, sys):
                self.sys = sys
            def __enter__(self):
                self.sys.start_continuous_acquisition()
                return self.sys
            def __exit__(self, exc_type, exc, tb):
                self.sys.stop_continuous_acquisition()
        return _Ctx(self)

    def read_voltages_numpy(self, max_samples: int = 1000):
        import numpy as np
        with self._lock:
            if not self._running:
                raise RuntimeError("System not running")
            channels_total = len(self.settings.devices) * 8
            data = np.empty((max_samples, channels_total), dtype=np.float32)
            for i in range(max_samples):
                base = ((self._counter + i) % 1000) / 1000.0
                for ch in range(channels_total):
                    data[i, ch] = (base * 2 - 1) + 0.01 * math.sin(ch + (self._counter + i) / 50.0)
            self._counter += max_samples
            return data

    def get_stats(self):
        return {
            "throughput_sps": sum(d.sample_rate for d in self.settings.devices),
            "devices": len(self.settings.devices),
        }

    # Stubs for future API completeness
    def configure_synchronization(self, cfg: dict):
        self._sync_cfg = cfg

    def get_sync_stats(self):
        return {"quality": 1.0, "error_count": 0}

    def get_device(self, device_id: int):
        return self.settings.devices[device_id]

    def get_system_info(self):
        return {"hardware": "synthetic", "device_count": len(self.settings.devices)}
