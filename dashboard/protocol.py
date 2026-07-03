"""Python mirror of firmware/include/protocol.h.

Speaks the identical line-based ASCII protocol used by both the STM32
firmware (over UART) and the native simulation binary (over stdio), so
this module -- and everything built on it -- doesn't care which one
it's talking to.
"""
from dataclasses import dataclass

MODE_POSITION = 0
MODE_SPEED = 1


@dataclass
class Telemetry:
    t_ms: int
    mode: int
    setpoint: float
    position: float
    speed: float
    error: float
    output: float


def parse_telemetry(line: str):
    """Parses one 'T,...' telemetry line. Returns None if malformed."""
    line = line.strip()
    if not line.startswith("T,"):
        return None
    parts = line.split(",")
    if len(parts) != 8:
        return None
    try:
        return Telemetry(
            t_ms=int(parts[1]),
            mode=int(parts[2]),
            setpoint=float(parts[3]),
            position=float(parts[4]),
            speed=float(parts[5]),
            error=float(parts[6]),
            output=float(parts[7]),
        )
    except ValueError:
        return None


def cmd_set_gains(kp: float, ki: float, kd: float) -> str:
    return f"K,{kp},{ki},{kd}\n"


def cmd_set_setpoint(value: float) -> str:
    return f"S,{value}\n"


def cmd_set_mode(mode: int) -> str:
    return f"M,{mode}\n"


def cmd_enable(enable: bool) -> str:
    return f"E,{1 if enable else 0}\n"


def cmd_reset() -> str:
    return "R\n"
