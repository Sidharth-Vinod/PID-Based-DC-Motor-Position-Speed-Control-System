#include "protocol.h"
#include <stdio.h>
#include <string.h>

Command protocol_parse_command(const char *line) {
    Command cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = CMD_INVALID;

    if (line == NULL || line[0] == '\0') {
        cmd.type = CMD_NONE;
        return cmd;
    }

    switch (line[0]) {
        case 'K': {
            float kp, ki, kd;
            if (sscanf(line, "K,%f,%f,%f", &kp, &ki, &kd) == 3) {
                cmd.type = CMD_SET_GAINS;
                cmd.kp = kp;
                cmd.ki = ki;
                cmd.kd = kd;
            }
            break;
        }
        case 'S': {
            float setpoint;
            if (sscanf(line, "S,%f", &setpoint) == 1) {
                cmd.type = CMD_SET_SETPOINT;
                cmd.setpoint = setpoint;
            }
            break;
        }
        case 'M': {
            int mode;
            if (sscanf(line, "M,%d", &mode) == 1 &&
                (mode == MODE_POSITION || mode == MODE_SPEED)) {
                cmd.type = CMD_SET_MODE;
                cmd.mode = mode;
            }
            break;
        }
        case 'E': {
            int enable;
            if (sscanf(line, "E,%d", &enable) == 1) {
                cmd.type = CMD_ENABLE;
                cmd.enable = enable ? 1 : 0;
            }
            break;
        }
        case 'R': {
            cmd.type = CMD_RESET;
            break;
        }
        default:
            break;
    }

    return cmd;
}

int protocol_format_telemetry(char *buf, size_t bufsize, const Telemetry *t) {
    int n = snprintf(buf, bufsize, "T,%lu,%d,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                      (unsigned long)t->t_ms, t->mode, t->setpoint,
                      t->position, t->speed, t->error, t->output);
    if (n < 0 || (size_t)n >= bufsize) {
        return -1;
    }
    return n;
}
