#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Line-based ASCII protocol shared by the STM32 firmware and the
 * native simulation harness, so the Python dashboard talks to either
 * one identically over a byte stream (real UART or a subprocess pipe).
 *
 * Host -> device commands (one per line):
 *   K,<kp>,<ki>,<kd>      set PID gains
 *   S,<value>             set setpoint (position counts, or speed counts/s)
 *   M,<0|1>               set mode: 0 = position hold, 1 = speed hold
 *   E,<0|1>               enable/disable motor output
 *   R                     reset integrator + zero position reference
 *
 * Device -> host telemetry (one per control tick):
 *   T,<t_ms>,<mode>,<setpoint>,<position>,<speed>,<error>,<output>
 */

typedef enum {
    CMD_NONE = 0,
    CMD_SET_GAINS,
    CMD_SET_SETPOINT,
    CMD_SET_MODE,
    CMD_ENABLE,
    CMD_RESET,
    CMD_INVALID
} CommandType;

typedef enum {
    MODE_POSITION = 0,
    MODE_SPEED = 1
} ControlMode;

typedef struct {
    CommandType type;
    float kp, ki, kd;
    float setpoint;
    int mode;
    int enable;
} Command;

typedef struct {
    uint32_t t_ms;
    int mode;
    float setpoint;
    float position;
    float speed;
    float error;
    float output;
} Telemetry;

/* Parses one NUL-terminated input line (no trailing newline required).
   Returns the recognized command, or CMD_INVALID on malformed input. */
Command protocol_parse_command(const char *line);

/* Formats one telemetry line into buf, including trailing '\n'.
   Returns the number of bytes written (excluding NUL), or -1 if buf
   was too small. */
int protocol_format_telemetry(char *buf, size_t bufsize, const Telemetry *t);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
