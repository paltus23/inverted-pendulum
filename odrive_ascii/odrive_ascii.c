#include "odrive_ascii.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

// Internal helpers
static int append_checksum(char *dst, size_t cap, const char *line_wo_cs)
{
    // dst receives: "<line_wo_cs> *NN"
    size_t n = strlen(line_wo_cs);
    if (n + 4 >= cap)
        return -1;
    uint8_t cs = odrv_ascii_checksum_xor(line_wo_cs);
    int m = snprintf(dst, cap, "%s *%u", line_wo_cs, (unsigned)cs);
    return (m >= 0 && (size_t)m < cap) ? m : -1;
}

static bool ends_with_crlf(const char *s)
{
    size_t n = strlen(s);
    return (n >= 2 && s[n - 2] == '\r' && s[n - 1] == '\n');
}

uint8_t odrv_ascii_checksum_xor(const char *s)
{
    uint8_t x = 0;
    for (; *s; ++s)
        x ^= (uint8_t)(*s);
    return x;
}

void odrv_ascii_init(odrv_ascii_t *cli, odrv_write_fn w, odrv_read_fn r)
{
    memset(cli, 0, sizeof(*cli));
    cli->write = w;
    cli->read = r;
    cli->io_timeout_ms = 50; // tweak as needed
    cli->use_checksum = false;
    cli->nl_tx[0] = '\r';
    cli->nl_tx[1] = '\n';
    cli->nl_tx[2] = '\0';
}

static ssize_t read_line_crlf(odrv_ascii_t *cli, char *out, size_t out_sz)
{
    // Read until CRLF or buffer full or timeout. Returns len (no CRLF), 0 on timeout, <0 on error.
    if (!out || out_sz == 0)
        return -1;
    size_t wr = 0;
    for (;;)
    {
        uint8_t ch;
        ssize_t r = cli->read(&ch, 1, cli->io_timeout_ms);
        if (r < 0)
            return r;
        if (r == 0)
        {
            // No data right now; consider timed-out if nothing read at all
            if (wr == 0)
                return 0;
            // keep waiting a bit more; you may choose to break if you want hard timeouts
            continue;
        }
        if (wr + 1 >= out_sz)
        { // reserve 1 for '\0'
            out[wr] = '\0';
            return -2; // overflow
        }
        out[wr++] = (char)ch;
        out[wr] = '\0';
        if (ends_with_crlf(out))
        {
            // strip CRLF
            out[wr - 2] = '\0';
            return (ssize_t)(wr - 2);
        }
    }
}

ssize_t odrv_ascii_cmd(odrv_ascii_t *cli, char *out, size_t out_sz, const char *fmt, ...)
{
    char core[256];
    va_list ap;
    va_start(ap, fmt);
    int nw = vsnprintf(core, sizeof(core), fmt, ap);
    va_end(ap);
    if (nw < 0 || (size_t)nw >= sizeof(core))
        return -1;

    char line[320];
    if (cli->use_checksum)
    {
        if (append_checksum(line, sizeof(line), core) < 0)
            return -1;
    }
    else
    {
        // just copy the core
        snprintf(line, sizeof(line), "%s", core);
    }

    // Append newline (ODrive accepts \r, \n, \r\n, or '!')
    char tx[384];
    int nfull = snprintf(tx, sizeof(tx), "%s%s", line, cli->nl_tx[0] ? cli->nl_tx : "\r\n");
    if (nfull < 0 || (size_t)nfull >= sizeof(tx))
        return -1;

    // Send
    ssize_t w = cli->write((const uint8_t *)tx, (size_t)nfull, cli->io_timeout_ms);
    if (w < 0)
        return w;

    // Some commands don't return anything; others do.
    // We'll try to read one line; if timeout occurs, treat as no-response, not an error.
    if (!out || out_sz == 0)
        return 0;
    ssize_t nr = read_line_crlf(cli, out, out_sz);
    if (nr == 0)
        return 0; // no reply (valid for many setpoint/system cmds)
    if (nr < 0)
        return nr; // read error/overflow
    return nr;     // length of reply
}

// ------------------- Wrappers -------------------

int odrv_read_property(odrv_ascii_t *cli, char *out, size_t out_sz, const char *property)
{
    if (!property)
        return -1;
    ssize_t n = odrv_ascii_cmd(cli, out, out_sz, "r %s", property);
    return (n < 0) ? (int)n : 0;
}

int odrv_write_property(odrv_ascii_t *cli, char *out, size_t out_sz, const char *property, const char *value)
{
    if (!property || !value)
        return -1;
    ssize_t n = odrv_ascii_cmd(cli, out, out_sz, "w %s %s", property, value);
    return (n < 0) ? (int)n : 0;
}

int odrv_set_position(odrv_ascii_t *cli, int motor, double position, double vel_lim, double torque_lim)
{
    // q m pos [vel_lim] [torque_lim]
    if (vel_lim == 0.0 && torque_lim == 0.0)
    {
        return (odrv_ascii_cmd(cli, NULL, 0, "q %d %.9g", motor, position) < 0) ? -1 : 0;
    }
    else if (torque_lim == 0.0)
    {
        return (odrv_ascii_cmd(cli, NULL, 0, "q %d %.9g %.9g", motor, position, vel_lim) < 0) ? -1 : 0;
    }
    else
    {
        return (odrv_ascii_cmd(cli, NULL, 0, "q %d %.9g %.9g %.9g", motor, position, vel_lim, torque_lim) < 0) ? -1 : 0;
    }
}

int odrv_set_position_ff(odrv_ascii_t *cli, int motor, double position, double vel_ff, double torque_ff)
{
    // p m pos [vel_ff] [torque_ff]
    if (vel_ff == 0.0 && torque_ff == 0.0)
        return (odrv_ascii_cmd(cli, NULL, 0, "p %d %.9g", motor, position) < 0) ? -1 : 0;
    else if (torque_ff == 0.0)
        return (odrv_ascii_cmd(cli, NULL, 0, "p %d %.9g %.9g", motor, position, vel_ff) < 0) ? -1 : 0;
    else
        return (odrv_ascii_cmd(cli, NULL, 0, "p %d %.9g %.9g %.9g", motor, position, vel_ff, torque_ff) < 0) ? -1 : 0;
}

int odrv_set_velocity(odrv_ascii_t *cli, int motor, double velocity, double torque_ff)
{
    // v m vel [tff]
    if (torque_ff == 0.0)
        return (odrv_ascii_cmd(cli, NULL, 0, "v %d %.9g", motor, velocity) < 0) ? -1 : 0;
    else
        return (odrv_ascii_cmd(cli, NULL, 0, "v %d %.9g %.9g", motor, velocity, torque_ff) < 0) ? -1 : 0;
}

int odrv_set_torque(odrv_ascii_t *cli, int motor, double torque)
{
    return (odrv_ascii_cmd(cli, NULL, 0, "c %d %.9g", motor, torque) < 0) ? -1 : 0;
}

static int parse_two_doubles(const char *s, double *a, double *b)
{
    if (!s || !a || !b)
        return -1;
    // Expect "pos vel"
    double aa, bb;
    // Allow leading/trailing whitespace
    if (sscanf(s, " %lf %lf ", &aa, &bb) == 2)
    {
        *a = aa;
        *b = bb;
        return 0;
    }
    return -2;
}

int odrv_get_position_and_velocity(odrv_ascii_t *cli, int motor, double *pos_out, double *vel_out)
{
    char buf[96];
    ssize_t n = odrv_ascii_cmd(cli, buf, sizeof(buf), "f %d", motor);
    if (n <= 0)
        return -1; // feedback must reply "pos vel"
    return parse_two_doubles(buf, pos_out, vel_out);
}

int odrv_update_watchdog(odrv_ascii_t *cli, int motor)
{
    return (odrv_ascii_cmd(cli, NULL, 0, "u %d", motor) < 0) ? -1 : 0;
}

int odrv_encoder_set_abs(odrv_ascii_t *cli, int motor, double abs_pos)
{
    return (odrv_ascii_cmd(cli, NULL, 0, "es %d %.9g", motor, abs_pos) < 0) ? -1 : 0;
}

int odrv_save(odrv_ascii_t *cli) { return (odrv_ascii_cmd(cli, NULL, 0, "ss") < 0) ? -1 : 0; }
int odrv_erase(odrv_ascii_t *cli) { return (odrv_ascii_cmd(cli, NULL, 0, "se") < 0) ? -1 : 0; }
int odrv_reboot(odrv_ascii_t *cli) { return (odrv_ascii_cmd(cli, NULL, 0, "sr") < 0) ? -1 : 0; }
int odrv_clear_errors(odrv_ascii_t *cli) { return (odrv_ascii_cmd(cli, NULL, 0, "sc") < 0) ? -1 : 0; }
