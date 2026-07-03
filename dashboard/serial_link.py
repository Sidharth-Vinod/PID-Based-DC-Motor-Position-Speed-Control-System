"""Transport backends for talking to the motor controller.

Both backends expose the same tiny interface (readline / write_line /
close), so the rest of the dashboard never needs to know whether it's
talking to a real STM32 over UART or to the native simulation binary
over a subprocess pipe.
"""
import queue
import subprocess
import threading

import protocol


class SerialBackend:
    """Real hardware over a UART (via USB-TTL adapter or ST-Link VCP)."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        import serial  # pyserial; imported lazily so --sim mode works without it

        self.ser = serial.Serial(port, baud, timeout=timeout)

    def readline(self):
        line = self.ser.readline()
        return line.decode("ascii", errors="ignore") if line else None

    def write_line(self, line: str):
        self.ser.write(line.encode("ascii"))

    def close(self):
        self.ser.close()


class SimulationBackend:
    """The native motor_sim binary, standing in for real hardware."""

    def __init__(self, binary_path: str, fast: bool = False):
        args = [binary_path] + (["--fast"] if fast else [])
        self.proc = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def readline(self):
        line = self.proc.stdout.readline()
        return line if line else None

    def write_line(self, line: str):
        self.proc.stdin.write(line)
        self.proc.stdin.flush()

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class TelemetryReader(threading.Thread):
    """Background thread that continuously parses telemetry lines and
    hands Telemetry objects to the consumer via a thread-safe queue,
    so the plotting/UI loop never blocks on I/O."""

    def __init__(self, link, out_queue: "queue.Queue[protocol.Telemetry]"):
        super().__init__(daemon=True)
        self.link = link
        self.out_queue = out_queue
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            try:
                line = self.link.readline()
            except (OSError, ValueError):
                break
            if not line:
                continue
            telem = protocol.parse_telemetry(line)
            if telem is not None:
                self.out_queue.put(telem)

    def stop(self):
        self._stop_event.set()
