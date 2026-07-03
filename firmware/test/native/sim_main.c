/*
 * Native PC simulation harness.
 *
 * Links the exact pid.c / protocol.c that also build into the STM32
 * firmware against a simulated DC motor plant (motor_model.c), so the
 * control law can be exercised and validated on a dev machine with no
 * hardware attached. Speaks the identical line protocol over
 * stdin/stdout that the real board speaks over UART, so the Python
 * dashboard can drive this binary as a drop-in stand-in for hardware.
 *
 * POSIX only (uses select() for non-blocking stdin).
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>

#include "pid.h"
#include "protocol.h"
#include "motor_model.h"

#define CONTROL_HZ 100
#define CONTROL_DT (1.0 / CONTROL_HZ)
#define SUPPLY_VOLTS 12.0

static int stdin_has_line(void) {
    fd_set fds;
    struct timeval tv = {0, 0};
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

int main(int argc, char **argv) {
    int fast = (argc > 1 && strcmp(argv[1], "--fast") == 0);

    MotorModel motor;
    motor_model_init_defaults(&motor);

    PidController pid;
    pid_init(&pid, /*kp=*/0.08f, /*ki=*/0.35f, /*kd=*/0.002f,
              /*out_min=*/-100.0f, /*out_max=*/100.0f, (float)CONTROL_DT);

    int mode = MODE_POSITION;
    float setpoint = 0.0f;
    int enabled = 1;

    setvbuf(stdout, NULL, _IOLBF, 0);

    char linebuf[128];
    size_t linelen = 0;
    uint32_t t_ms = 0;

    for (;;) {
        /* Drain any pending commands from stdin without blocking. */
        while (stdin_has_line()) {
            char c;
            ssize_t n = read(STDIN_FILENO, &c, 1);
            if (n <= 0) break;
            if (c == '\n') {
                linebuf[linelen] = '\0';
                Command cmd = protocol_parse_command(linebuf);
                switch (cmd.type) {
                    case CMD_SET_GAINS:
                        pid_set_gains(&pid, cmd.kp, cmd.ki, cmd.kd);
                        break;
                    case CMD_SET_SETPOINT:
                        setpoint = cmd.setpoint;
                        break;
                    case CMD_SET_MODE:
                        mode = cmd.mode;
                        pid_reset(&pid);
                        break;
                    case CMD_ENABLE:
                        enabled = cmd.enable;
                        if (!enabled) pid_reset(&pid);
                        break;
                    case CMD_RESET:
                        motor.position = 0.0;
                        pid_reset(&pid);
                        break;
                    default:
                        break;
                }
                linelen = 0;
            } else if (linelen < sizeof(linebuf) - 1) {
                linebuf[linelen++] = c;
            }
        }

        double measurement = (mode == MODE_POSITION)
            ? motor_model_position_counts(&motor)
            : motor_model_speed_counts_per_sec(&motor);

        float output = enabled ? pid_step(&pid, setpoint, (float)measurement) : 0.0f;
        double voltage = enabled ? (output / 100.0) * SUPPLY_VOLTS : 0.0;

        motor_model_step(&motor, voltage, CONTROL_DT);

        Telemetry telem = {
            .t_ms = t_ms,
            .mode = mode,
            .setpoint = setpoint,
            .position = (float)motor_model_position_counts(&motor),
            .speed = (float)motor_model_speed_counts_per_sec(&motor),
            .error = setpoint - (float)measurement,
            .output = output,
        };
        char out[128];
        int n = protocol_format_telemetry(out, sizeof(out), &telem);
        if (n > 0) {
            fwrite(out, 1, (size_t)n, stdout);
        }

        t_ms += (uint32_t)(CONTROL_DT * 1000);
        if (!fast) usleep((useconds_t)(CONTROL_DT * 1e6));
    }

    return 0;
}
