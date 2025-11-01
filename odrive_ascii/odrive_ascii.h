#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // ---- Transport abstraction --------------------------------------------------
    // Return number of bytes written/read, or negative on error.
    // read_fn must block (up to timeout_ms) until at least one byte is available or timeout.
    // It may return 0 on timeout.
    typedef ssize_t (*odrv_write_fn)(const uint8_t *buf, size_t len, uint32_t timeout_ms);
    typedef ssize_t (*odrv_read_fn)(uint8_t *buf, size_t len, uint32_t timeout_ms);

    typedef struct
    {
        odrv_write_fn write;
        odrv_read_fn read;
        uint32_t io_timeout_ms;
        bool use_checksum; // if true, append *CS and expect checksum in reply (per spec)
        char nl_tx[3];     // newline to send, defaults to "\r\n"
    } odrv_ascii_t;

    // Minimal init; if nl_tx is empty, it will default to "\r\n"
    void odrv_ascii_init(odrv_ascii_t *cli, odrv_write_fn w, odrv_read_fn r);

    // Utilities
    uint8_t odrv_ascii_checksum_xor(const char *line_without_star); // xor of all chars
    // Send one formatted command (printf-style for the command portion), optionally with checksum,
    // then read a single CRLF-terminated line into out (stripped of CRLF). Returns >=0 length, or <0 on error.
    ssize_t odrv_ascii_cmd(odrv_ascii_t *cli, char *out, size_t out_sz, const char *fmt, ...);

    // Convenience wrappers --------------------------------------------------------

    // Parameter read: r <property>      -> returns value text in out
    int odrv_read_property(odrv_ascii_t *cli, char *out, size_t out_sz, const char *property);

    // Parameter write: w <property> <value> -> returns reply (often empty; may be "ok" if FW does so)
    int odrv_write_property(odrv_ascii_t *cli, char *out, size_t out_sz, const char *property, const char *value);

    // Motion / control commands (watchdog auto-updates per spec for q/p/v/c)
    int odrv_set_position(odrv_ascii_t *cli, int motor, double position, double vel_lim, double torque_lim);  // q m pos [vellim] [torquelim]
    int odrv_set_position_ff(odrv_ascii_t *cli, int motor, double position, double vel_ff, double torque_ff); // p m pos [vff] [tff]
    int odrv_set_velocity(odrv_ascii_t *cli, int motor, double velocity, double torque_ff);                   // v m vel [tff]
    int odrv_set_torque(odrv_ascii_t *cli, int motor, double torque);                                         // c m tq

    // Feedback: f m -> "pos vel"
    int odrv_get_position_and_velocity(odrv_ascii_t *cli, int motor, double *pos_out, double *vel_out);

    // Watchdog update only (no setpoint change): u m
    int odrv_update_watchdog(odrv_ascii_t *cli, int motor);

    // Encoder absolute set: es m abs_pos
    int odrv_encoder_set_abs(odrv_ascii_t *cli, int motor, double abs_pos);

    // System commands: ss/se/sr/sc
    int odrv_save(odrv_ascii_t *cli);
    int odrv_erase(odrv_ascii_t *cli);
    int odrv_reboot(odrv_ascii_t *cli);
    int odrv_clear_errors(odrv_ascii_t *cli);

#ifdef __cplusplus
}
#endif
