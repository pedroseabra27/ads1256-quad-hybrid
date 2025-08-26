from .system import (
    ADS1256QuadSystem,
    SystemSettings,
    DeviceSettings,
    ChannelSettings,
    create_default_system,
)

__all__ = [
    "ADS1256QuadSystem",
    "SystemSettings",
    "DeviceSettings",
    "ChannelSettings",
    "create_default_system",
]

try:
    from ._core import CoreSystem  # Cython hardware-backed system
    __all__.append("CoreSystem")
except Exception:  # pragma: no cover
    pass
