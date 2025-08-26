## System & Device Configuration

### SystemSettings (planned Python dataclass)

| Field | Type | Description |
|-------|------|-------------|
| devices | list[DeviceSettings] | Four device configs |
| sync_mode | str | 'hardware' or 'software' |
| use_dma | bool | Enable DMA path (future) |
| real_time | bool | Request RT scheduling |
| rt_priority | int | FIFO priority (1-99) |
| cpu_affinity | list[int] | Core pinning (optional) |
| stats_interval_s | float | Perf stats cadence |

### DeviceSettings

| Field | Type | Description |
|-------|------|-------------|
| device_id | int | 0..3 |
| spi_bus | int | 0 or 1 |
| chip_select | int | CS line (0/1) on that bus |
| sample_rate | int | SPS per device (<=30000) |
| buffer_size | int | Bytes reserved per device |
| auto_calibration | bool | Perform self-cal after init |
| channels | list[ChannelSettings] | Per-channel options |

### ChannelSettings

| Field | Type | Description |
|-------|------|-------------|
| enabled | bool | Acquire channel yes/no |
| gain | int | PGA gain (1,2,4,8,16,32,64) |
| input_mode | str | 'single' or 'diff' |
| label | str | User label |

### Sync Configuration (dictionary form)

```
{
  "mode": "hardware",
  "sync_frequency": 1000,
  "tolerance_us": 10,
  "gpio_sync_pin": 18,
  "recovery_enabled": true
}
```

### Environment / System Tuning (Linux)

Add to `/etc/security/limits.conf` for RT priority:
```
username soft rtprio 99
username hard rtprio 99
```

Governor:
```
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

Optional `/boot/config.txt` (Pi):
```
force_turbo=1
dtparam=spi=on
```
