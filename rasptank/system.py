"""Host status readings from server/info.py (CPU/GPU temperature, CPU/RAM/swap use)."""

import subprocess

CPU_TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp"


def cpu_temperature(path=CPU_TEMP_PATH):
    """CPU temperature in Celsius, rounded to 0.1."""
    with open(path) as handle:
        return round(float(handle.read().strip()) / 1000, 1)


def gpu_temperature(command=("/opt/vc/bin/vcgencmd", "measure_temp")):
    """GPU temperature string as printed by vcgencmd, e.g. '45.2'C'. Needs Pi firmware tools."""
    output = subprocess.run(command, capture_output=True, text=True, check=True).stdout
    return output.strip().replace("temp=", "")


def _psutil():
    """Import psutil lazily so importing this module needs no extra dependency."""
    import psutil
    return psutil


def cpu_percent():
    """CPU utilisation percent since the previous psutil call."""
    return _psutil().cpu_percent()


def ram_percent():
    """RAM utilisation percent."""
    return _psutil().virtual_memory().percent


def swap_percent():
    """Swap utilisation percent."""
    return _psutil().swap_memory().percent
