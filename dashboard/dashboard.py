#!/usr/bin/env python3
"""Live tuning and data-logging dashboard for the PID motor controller.

Talks to either real hardware (--port) or the native simulation binary
(--sim, the default) over the shared line protocol in protocol.py, and
gives a live view of the step response plus sliders to tune Kp/Ki/Kd
and the setpoint while the loop is running.

Examples:
    python3 dashboard.py --sim
    python3 dashboard.py --port /dev/ttyUSB0
    python3 dashboard.py --sim --headless --duration 10 --plot-out step.png
"""
import argparse
import csv
import os
import queue
import time
from collections import deque
from datetime import datetime

import matplotlib

import protocol
from serial_link import SerialBackend, SimulationBackend, TelemetryReader

DEFAULT_SIM_BIN = os.path.join(
    os.path.dirname(__file__), "..", "firmware", "test", "native", "motor_sim"
)

HISTORY_SECONDS = 12
CONTROL_HZ = 100
HISTORY_LEN = HISTORY_SECONDS * CONTROL_HZ


class History:
    def __init__(self, maxlen=None):
        self.t = deque(maxlen=maxlen)
        self.setpoint = deque(maxlen=maxlen)
        self.position = deque(maxlen=maxlen)
        self.speed = deque(maxlen=maxlen)
        self.error = deque(maxlen=maxlen)
        self.output = deque(maxlen=maxlen)

    def append(self, telem: protocol.Telemetry):
        self.t.append(telem.t_ms / 1000.0)
        self.setpoint.append(telem.setpoint)
        self.position.append(telem.position)
        self.speed.append(telem.speed)
        self.error.append(telem.error)
        self.output.append(telem.output)


def make_link(args):
    if args.port:
        return SerialBackend(args.port, args.baud)
    return SimulationBackend(args.sim_bin, fast=args.fast_sim)


def make_logger(log_dir):
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, f"run_{datetime.now():%Y%m%d_%H%M%S}.csv")
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["t_ms", "mode", "setpoint", "position", "speed", "error", "output"])
    return f, writer, path


def run_headless(args):
    link = make_link(args)
    q: "queue.Queue[protocol.Telemetry]" = queue.Queue()
    reader = TelemetryReader(link, q)
    reader.start()

    log_file, writer, log_path = make_logger(args.log_dir)
    hist = History()  # unbounded: headless mode captures the whole run, not a rolling window

    link.write_line(protocol.cmd_set_gains(args.kp, args.ki, args.kd))
    link.write_line(protocol.cmd_set_mode(args.mode))
    link.write_line(protocol.cmd_enable(True))
    link.write_line(protocol.cmd_set_setpoint(args.setpoint))

    # In --fast sim mode telemetry timestamps race far ahead of wall
    # clock, so stop once *simulated* time reaches the requested
    # duration; the wall-clock deadline is just a safety net.
    wall_deadline = time.monotonic() + max(args.duration, 30.0)
    while time.monotonic() < wall_deadline:
        try:
            telem = q.get(timeout=0.5)
        except queue.Empty:
            continue
        hist.append(telem)
        writer.writerow([telem.t_ms, telem.mode, telem.setpoint, telem.position,
                          telem.speed, telem.error, telem.output])
        if telem.t_ms / 1000.0 >= args.duration:
            break

    reader.stop()
    link.close()
    log_file.close()
    print(f"Logged {len(hist.t)} samples to {log_path}")

    if args.plot_out:
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
        axes[0].plot(hist.t, hist.setpoint, label="setpoint", linestyle="--")
        axes[0].plot(hist.t, hist.position, label="position")
        axes[0].set_ylabel("position (counts)")
        axes[0].legend(loc="lower right")

        axes[1].plot(hist.t, hist.speed, label="speed", color="tab:orange")
        axes[1].set_ylabel("speed (counts/s)")

        axes[2].plot(hist.t, hist.error, label="error", color="tab:red")
        axes[2].plot(hist.t, hist.output, label="output %", color="tab:green")
        axes[2].set_ylabel("error / output")
        axes[2].set_xlabel("time (s)")
        axes[2].legend(loc="lower right")

        fig.suptitle(f"Step response (Kp={args.kp}, Ki={args.ki}, Kd={args.kd})")
        fig.tight_layout()
        fig.savefig(args.plot_out, dpi=120)
        print(f"Saved plot to {args.plot_out}")


def run_interactive(args):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, Slider

    link = make_link(args)
    q: "queue.Queue[protocol.Telemetry]" = queue.Queue()
    reader = TelemetryReader(link, q)
    reader.start()

    log_file, writer, log_path = make_logger(args.log_dir)
    hist = History(HISTORY_LEN)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    plt.subplots_adjust(left=0.09, right=0.78, bottom=0.08, top=0.93, hspace=0.3)

    (line_setpoint,) = axes[0].plot([], [], "--", label="setpoint")
    (line_position,) = axes[0].plot([], [], label="position")
    axes[0].set_ylabel("position (counts)")
    axes[0].legend(loc="upper left")

    (line_speed,) = axes[1].plot([], [], color="tab:orange", label="speed")
    axes[1].set_ylabel("speed (counts/s)")
    axes[1].legend(loc="upper left")

    (line_error,) = axes[2].plot([], [], color="tab:red", label="error")
    (line_output,) = axes[2].plot([], [], color="tab:green", label="output %")
    axes[2].set_ylabel("error / output")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper left")

    fig.suptitle("PID DC Motor Control - Live Tuning Dashboard")

    slider_ax_kp = fig.add_axes([0.83, 0.80, 0.13, 0.03])
    slider_ax_ki = fig.add_axes([0.83, 0.73, 0.13, 0.03])
    slider_ax_kd = fig.add_axes([0.83, 0.66, 0.13, 0.03])
    slider_ax_sp = fig.add_axes([0.83, 0.59, 0.13, 0.03])

    s_kp = Slider(slider_ax_kp, "Kp", 0.0, 2.0, valinit=args.kp)
    s_ki = Slider(slider_ax_ki, "Ki", 0.0, 5.0, valinit=args.ki)
    s_kd = Slider(slider_ax_kd, "Kd", 0.0, 0.05, valinit=args.kd)
    s_sp = Slider(slider_ax_sp, "Setpoint", -3000.0, 3000.0, valinit=args.setpoint)

    radio_ax = fig.add_axes([0.83, 0.42, 0.13, 0.12])
    radio_mode = RadioButtons(radio_ax, ("Position", "Speed"),
                               active=args.mode)

    btn_enable_ax = fig.add_axes([0.83, 0.33, 0.13, 0.05])
    btn_enable = Button(btn_enable_ax, "Enable/Disable")

    btn_reset_ax = fig.add_axes([0.83, 0.26, 0.13, 0.05])
    btn_reset = Button(btn_reset_ax, "Reset")

    state = {"enabled": True, "mode": args.mode}

    def send_gains(_=None):
        link.write_line(protocol.cmd_set_gains(s_kp.val, s_ki.val, s_kd.val))

    def send_setpoint(_=None):
        link.write_line(protocol.cmd_set_setpoint(s_sp.val))

    def on_mode(label):
        state["mode"] = protocol.MODE_POSITION if label == "Position" else protocol.MODE_SPEED
        link.write_line(protocol.cmd_set_mode(state["mode"]))

    def on_enable(_):
        state["enabled"] = not state["enabled"]
        link.write_line(protocol.cmd_enable(state["enabled"]))

    def on_reset(_):
        link.write_line(protocol.cmd_reset())
        hist.t.clear()
        hist.setpoint.clear()
        hist.position.clear()
        hist.speed.clear()
        hist.error.clear()
        hist.output.clear()

    s_kp.on_changed(send_gains)
    s_ki.on_changed(send_gains)
    s_kd.on_changed(send_gains)
    s_sp.on_changed(send_setpoint)
    radio_mode.on_clicked(on_mode)
    btn_enable.on_clicked(on_enable)
    btn_reset.on_clicked(on_reset)

    link.write_line(protocol.cmd_set_gains(args.kp, args.ki, args.kd))
    link.write_line(protocol.cmd_set_mode(args.mode))
    link.write_line(protocol.cmd_enable(True))
    link.write_line(protocol.cmd_set_setpoint(args.setpoint))

    def update(_frame):
        drained = 0
        while True:
            try:
                telem = q.get_nowait()
            except queue.Empty:
                break
            hist.append(telem)
            writer.writerow([telem.t_ms, telem.mode, telem.setpoint, telem.position,
                              telem.speed, telem.error, telem.output])
            drained += 1
        if drained == 0:
            return (line_setpoint, line_position, line_speed, line_error, line_output)

        line_setpoint.set_data(hist.t, hist.setpoint)
        line_position.set_data(hist.t, hist.position)
        line_speed.set_data(hist.t, hist.speed)
        line_error.set_data(hist.t, hist.error)
        line_output.set_data(hist.t, hist.output)

        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        return (line_setpoint, line_position, line_speed, line_error, line_output)

    from matplotlib.animation import FuncAnimation

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        reader.stop()
        link.close()
        log_file.close()
        print(f"Logged samples to {log_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Serial port for real hardware, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sim-bin", default=DEFAULT_SIM_BIN,
                         help="Path to the native motor_sim binary (used when --port is omitted)")
    parser.add_argument("--kp", type=float, default=0.08)
    parser.add_argument("--ki", type=float, default=0.35)
    parser.add_argument("--kd", type=float, default=0.002)
    parser.add_argument("--setpoint", type=float, default=1000.0)
    parser.add_argument("--mode", type=int, choices=[0, 1], default=protocol.MODE_POSITION,
                         help="0 = position hold, 1 = speed hold")
    parser.add_argument("--log-dir", default=os.path.join(os.path.dirname(__file__), "logs"))
    parser.add_argument("--headless", action="store_true",
                         help="Run without opening a window: log to CSV (and optionally a PNG) for a fixed duration")
    parser.add_argument("--duration", type=float, default=10.0,
                         help="Seconds to run in --headless mode")
    parser.add_argument("--fast-sim", action="store_true",
                         help="Run the simulation binary unthrottled instead of paced in real time. "
                              "Only useful for --headless batch runs with a fixed --setpoint/--mode from "
                              "startup (set via CLI flags, not sliders) since commands sent after the "
                              "process starts can race the unthrottled tick loop.")
    parser.add_argument("--plot-out", help="If set with --headless, save a step-response PNG here")
    args = parser.parse_args()

    if args.headless:
        run_headless(args)
    else:
        run_interactive(args)


if __name__ == "__main__":
    main()
