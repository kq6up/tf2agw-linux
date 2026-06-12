/*
 * tf2agw: experimental Linux PTY DED/TF-ish to AGWPE/Dire Wolf bridge.
 *
 * This is a clean, minimal prototype inspired by LU7DID's tf2agw design notes,
 * not a port of the recovered Delphi DLL source.
 *
 * Build: cc -Wall -Wextra -O2 -o tf2agw tf2agw.c
 * Usage: ./tf2agw -s /tmp/tf2agw -a 127.0.0.1 -p 8000 -c KQ6UP-1 -v
 *
 * Point LinFBB's TF/WA8DED device at /tmp/tf2agw.
 * This is a protocol spike, not production-ready.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define AGW_HDR_LEN 36
#define MAX_CH 33  /* channels 0..32; FBB polls 1..nb_voies plus 0 */
#define MAX_BUF 2048
#define MAX_CALL 16
#define DEFAULT_AGW_HOST "127.0.0.1"
#define DEFAULT_AGW_PORT "8000"
#define DEFAULT_SYMLINK "/tmp/tf2agw"

struct agw_hdr {
    uint8_t port;
    uint8_t r1;
    uint8_t r2;
    uint8_t r3;
    char kind;
    uint8_t r4;
    uint8_t pid;
    uint8_t r5;
    char from[10];
    char to[10];
    int32_t len;
    int32_t user;
} __attribute__((packed));

struct chan {
    bool used;
    int port;
    int st;     /* TF L2 state: 0 disc, 1 link setup, 3 disc req, 4 connected */
    char my[MAX_CALL];
    char peer[MAX_CALL];
};

static volatile sig_atomic_t stop_flag = 0;
static int verbose = 0;
static struct chan ch[MAX_CH];
static char mycall[MAX_CALL] = "";
static int hostmode = 0;       /* 0 = terminal mode, 1 = WA8DED hostmode */

/* terminal-mode (pre-hostmode) line discipline state, TF-style */
static bool t_esc = false;     /* current line started with ESC -> command */
static bool t_echo = true;     /* TF Epar default: echo on                 */
static int  t_cur = 0;         /* channel selected with the S command      */
static char t_line[MAX_BUF];
static size_t t_lp = 0;

static void die(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(1);
}

static void logv(const char *fmt, ...) {
    if (!verbose) return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void on_signal(int sig) { (void)sig; stop_flag = 1; }

static int set_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void normalize_call(char *s) {
    size_t n = strlen(s);
    while (n && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' ' || s[n-1] == '\t')) s[--n] = 0;
    while (*s == ' ' || *s == '\t') memmove(s, s+1, strlen(s));
    for (char *p = s; *p; ++p) if (*p >= 'a' && *p <= 'z') *p -= 32;
    s[MAX_CALL-1] = 0;
    /*
     * Strip an explicit -0 SSID: "KQ6UP-0" and "KQ6UP" are the same AX.25
     * station, but Direwolf matches sessions with strcmp, and its radio-side
     * parser renders SSID 0 without the suffix.  FBB sends "I CALL-0", so
     * without this, outgoing UAs never match the session ("not for me")
     * and incoming connects never match the registered callsign.
     */
    n = strlen(s);
    if (n >= 2 && s[n-2] == '-' && s[n-1] == '0') s[n-2] = 0;
}

static void pack_call(char out[10], const char *call) {
    memset(out, 0, 10);
    strncpy(out, call, 9);
}

static int connect_tcp(const char *host, const char *port) {
    struct addrinfo hints, *res = NULL, *rp;
    int fd = -1;
    memset(&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_UNSPEC;
    int e = getaddrinfo(host, port, &hints, &res);
    if (e) die("getaddrinfo(%s:%s): %s", host, port, gai_strerror(e));
    for (rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;
        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) break;
        close(fd); fd = -1;
    }
    freeaddrinfo(res);
    if (fd < 0) die("could not connect to AGW server %s:%s", host, port);
    set_nonblock(fd);
    return fd;
}

static int write_all(int fd, const void *buf, size_t len) {
    const uint8_t *p = buf;
    while (len) {
        ssize_t n = write(fd, p, len);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) { usleep(1000); continue; }
            return -1;
        }
        p += n; len -= (size_t)n;
    }
    return 0;
}

static void agw_send(int fd, int port, char kind, uint8_t pid, const char *from, const char *to, const void *payload, int32_t len) {
    struct agw_hdr h;
    memset(&h, 0, sizeof(h));
    h.port = (uint8_t)port;
    h.kind = kind;
    h.pid = pid;
    pack_call(h.from, from ? from : "");
    pack_call(h.to, to ? to : "");
    h.len = len;
    uint8_t *buf = malloc(AGW_HDR_LEN + (len > 0 ? len : 0));
    if (!buf) die("malloc");
    memcpy(buf, &h, AGW_HDR_LEN);
    if (len > 0 && payload) memcpy(buf + AGW_HDR_LEN, payload, len);
    logv("AGW send kind=%c port=%d from=%s to=%s len=%d", kind, port, from ? from : "", to ? to : "", len);
    if (write_all(fd, buf, AGW_HDR_LEN + (len > 0 ? len : 0)) < 0) perror("write agw");
    free(buf);
}

/*
 * AGW connect: plain 'C' frame, or 'v' frame when digipeaters are given.
 * Per the AGW spec the via path is not payload on 'C'; it is a separate
 * 'v' frame whose payload is 1 count byte + count * 10-byte padded calls.
 */
static void agw_send_connect(int fd, int port, const char *from, const char *to, const char *via) {
    if (via && *via) {
        uint8_t payload[1 + 7 * 10];
        int nd = 0;
        char tmp[256];
        strncpy(tmp, via, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = 0;
        for (char *tok = strtok(tmp, " ,\t"); tok && nd < 7; tok = strtok(NULL, " ,\t")) {
            if (!strcasecmp(tok, "via") || !strcasecmp(tok, "v")) continue;
            char call[MAX_CALL];
            strncpy(call, tok, sizeof(call) - 1);
            call[sizeof(call) - 1] = 0;
            normalize_call(call);
            pack_call((char *)payload + 1 + nd * 10, call);
            nd++;
        }
        payload[0] = (uint8_t)nd;
        agw_send(fd, port, 'v', 0xF0, from, to, payload, 1 + nd * 10);
    } else {
        agw_send(fd, port, 'C', 0xF0, from, to, NULL, 0);
    }
}

static void host_line(int pty, int channel, const char *fmt, ...) {
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    (void)channel;  /* terminal mode is plain text; channels exist only in hostmode */
    write_all(pty, line, strlen(line));
    write_all(pty, "\r\n", 2);
    logv("HOST <= ch=%d %s", channel, line);
}

static int first_free_channel(void) {
    for (int i = 1; i < MAX_CH; i++) if (!ch[i].used) return i;
    return -1;
}

static int find_channel(const char *my, const char *peer, int port) {
    for (int i = 1; i < MAX_CH; i++) {
        if (ch[i].used && ch[i].port == port && !strcasecmp(ch[i].my, my) && !strcasecmp(ch[i].peer, peer)) return i;
    }
    return -1;
}

/* ------------------------------------------------------------------------
 * WA8DED hostmode.
 *
 * After FBB sends JHOST1 (a terminal-mode command), framing goes binary:
 *   host -> tnc : [chan][1=command, 0=info][len-1][len bytes]
 *   tnc -> host : [chan][code][payload]
 * Response codes: 0 ok (nothing follows), 1 ok + NUL-terminated message,
 * 2 failure + message, 3 link status + message, 4/5 monitor header,
 * 6 monitor data [len-1][bytes], 7 connected data [len-1][bytes].
 *
 * Every host frame gets exactly one response (stop-and-wait).  Link
 * status and received data are queued per channel and delivered only in
 * answer to G polls.  Reference implementation: tfkiss tfa.c/tfb.c/tfc.c.
 * ---------------------------------------------------------------------- */

struct qitem {
    struct qitem *next;
    uint8_t code;               /* 3 = link status, 7 = connected data */
    size_t len;
    uint8_t data[];
};

static struct qitem *qhead[MAX_CH], *qtail[MAX_CH];

static void q_push(int chan, uint8_t code, const void *data, size_t len) {
    if (chan < 0 || chan >= MAX_CH) return;
    struct qitem *it = malloc(sizeof(*it) + len);
    if (!it) return;
    it->next = NULL;
    it->code = code;
    it->len = len;
    memcpy(it->data, data, len);
    if (qtail[chan]) qtail[chan]->next = it; else qhead[chan] = it;
    qtail[chan] = it;
}

static void q_flush(int chan) {
    struct qitem *it = qhead[chan];
    while (it) { struct qitem *nx = it->next; free(it); it = nx; }
    qhead[chan] = qtail[chan] = NULL;
}

/* pop first item matching want: 'A' any, 'I' info (data), 'S' status */
static struct qitem *q_pop(int chan, char want) {
    struct qitem **pp = &qhead[chan];
    while (*pp) {
        struct qitem *it = *pp;
        bool is_status = (it->code != 7);
        if (want == 'A' || (want == 'S' && is_status) || (want == 'I' && !is_status)) {
            *pp = it->next;
            if (qtail[chan] == it) {
                qtail[chan] = NULL;
                for (struct qitem *t = qhead[chan]; t; t = t->next) qtail[chan] = t;
            }
            return it;
        }
        pp = &it->next;
    }
    return NULL;
}

/* count queued items of one class: 'S' status, 'I' info (data), 'A' any */
static int q_count(int chan, char want) {
    int n = 0;
    for (struct qitem *it = qhead[chan]; it; it = it->next) {
        bool is_status = (it->code != 7);
        if (want == 'A' || (want == 'S' && is_status) || (want == 'I' && !is_status)) n++;
    }
    return n;
}

static void q_status(int chan, const char *fmt, ...) {
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    q_push(chan, 3, msg, strlen(msg) + 1);   /* NUL included */
    logv("HM queue ch=%d status: %s", chan, msg);
}

static void q_data(int chan, const uint8_t *p, size_t len) {
    while (len) {                            /* data frames carry at most 256 bytes */
        size_t n = len > 256 ? 256 : len;
        q_push(chan, 7, p, n);
        p += n;
        len -= n;
    }
}

/* [chan][0] : command ok / nothing to report */
static void hm_ok(int pty, int chan) {
    uint8_t b[2] = { (uint8_t)chan, 0 };
    write_all(pty, b, 2);
}

/* [chan][code][msg NUL] : code 1 ok+msg, 2 fail+msg, 3 link status */
static void hm_msg(int pty, int chan, uint8_t code, const char *fmt, ...) {
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    uint8_t b[2] = { (uint8_t)chan, code };
    write_all(pty, b, 2);
    write_all(pty, msg, strlen(msg) + 1);
    logv("HM <= ch=%d code=%u %s", chan, code, msg);
}

/* answer one G poll on one channel */
static void hm_poll(int pty, int chan, char want) {
    struct qitem *it = q_pop(chan, want);
    if (!it) { hm_ok(pty, chan); return; }
    if (it->code == 7) {
        uint8_t b[3] = { (uint8_t)chan, 7, (uint8_t)(it->len - 1) };
        write_all(pty, b, 3);
        write_all(pty, it->data, it->len);
        logv("HM <= ch=%d data %zu bytes", chan, it->len);
    } else {
        uint8_t b[2] = { (uint8_t)chan, it->code };
        write_all(pty, b, 2);
        write_all(pty, it->data, it->len);   /* NUL-terminated already */
        logv("HM <= ch=%d code=%u %s", chan, it->code, (char *)it->data);
    }
    free(it);
}

static int hm_st = 0;                        /* binary frame parser state */
static uint8_t hm_chan, hm_flag;
static size_t hm_need, hm_got;
static uint8_t hm_buf[260];

static void hm_command(int agwfd, int pty, int chan, char *cmd) {
    logv("HM => ch=%d cmd: %s", chan, cmd);

    char c = cmd[0];
    if (c >= 'a' && c <= 'z') c -= 32;
    char *args = cmd + 1;
    while (*args == ' ' || *args == '\t') args++;

    /* extended hostmode poll (DG3DBI): G on channel 255 returns a       */
    /* NUL-terminated list of channels with pending output, numbers + 1  */
    if (chan == 0xFF) {
        if (c == 'G') {
            uint8_t resp[MAX_CH + 3];
            size_t k = 0;
            resp[k++] = 0xFF;
            resp[k++] = 1;
            for (int j = 0; j < MAX_CH; j++)
                if (qhead[j]) resp[k++] = (uint8_t)(j + 1);
            resp[k++] = 0;
            write_all(pty, resp, k);
        } else hm_msg(pty, chan, 2, "INVALID CHANNEL NUMBER");
        return;
    }
    if (chan >= MAX_CH) { hm_msg(pty, chan, 2, "INVALID CHANNEL NUMBER"); return; }

    if (!strncasecmp(cmd, "JHOST", 5)) {
        if (cmd[5] == '0') {                 /* back to terminal mode */
            hm_ok(pty, chan);                /* ack while still in hostmode */
            hostmode = 0;
            hm_st = 0;
            t_lp = 0;
            t_esc = false;
            for (int j = 0; j < MAX_CH; j++) q_flush(j);
            logv("leaving hostmode");
        } else if (cmd[5] == '1') hm_ok(pty, chan);  /* already in hostmode */
        else hm_msg(pty, chan, 1, "1");      /* bare JHOST query */
        return;
    }

    switch (c) {
    case 'G':                                /* G = all, G0 = info, G1 = status */
        hm_poll(pty, chan, *args == '0' ? 'I' : *args == '1' ? 'S' : 'A');
        return;
    case 'C': {
        if (chan == 0) { hm_msg(pty, chan, 2, "INVALID CHANNEL NUMBER"); return; }
        if (ch[chan].used) { hm_msg(pty, chan, 2, "ALREADY CONNECTED"); return; }
        int port = 0;
        char call[MAX_CALL] = "";
        char a[64] = "", b[64] = "", rest[256] = "", viabuf[336] = "";
        sscanf(args, "%63s %63s %255[^\r\n]", a, b, rest);
        if (a[0] && b[0] && a[0] >= '0' && a[0] <= '9') {
            port = atoi(a);
            strncpy(call, b, sizeof(call) - 1);
            strncpy(viabuf, rest, sizeof(viabuf) - 1);
        } else {
            strncpy(call, a, sizeof(call) - 1);
            if (b[0]) snprintf(viabuf, sizeof(viabuf), "%s %s", b, rest);
        }
        normalize_call(call);
        if (!call[0]) { hm_msg(pty, chan, 2, "INVALID COMMAND"); return; }
        ch[chan].used = true;                /* hostmode: FBB picks the channel */
        ch[chan].st = 1;                     /* link setup */
        ch[chan].port = port;
        strncpy(ch[chan].my, mycall, sizeof(ch[chan].my) - 1);
        strncpy(ch[chan].peer, call, sizeof(ch[chan].peer) - 1);
        agw_send_connect(agwfd, port, mycall, call, viabuf);
        hm_ok(pty, chan);                    /* "(n) CONNECTED to" arrives via poll */
        return;
    }
    case 'D':
        if (chan > 0 && ch[chan].used) {
            agw_send(agwfd, ch[chan].port, 'd', 0, ch[chan].my, ch[chan].peer, NULL, 0);
            ch[chan].st = 3;                 /* disconnect request */
            hm_ok(pty, chan);                /* "(n) DISCONNECTED fm" follows */
        } else hm_msg(pty, chan, 2, "CHANNEL NOT CONNECTED");
        return;
    case 'I':
        if (*args) {
            strncpy(mycall, args, sizeof(mycall) - 1);
            mycall[sizeof(mycall) - 1] = 0;
            normalize_call(mycall);
            if (*mycall) agw_send(agwfd, 0, 'X', 0, mycall, "", NULL, 0);
            hm_ok(pty, chan);
        } else hm_msg(pty, chan, 1, "%s", mycall);
        return;
    case 'L':
        /*
         * TF Lcmd hostmode reply (tfkiss tfb.c):
         *   ch 0  : "<status msgs pending> <monitor frames pending>"
         *   ch >0 : "<status pending> <data pending> <unsent> <unacked>
         *            <retries> <L2 state>"
         * FBB polls with L and only fetches (G) when the first two
         * counts are nonzero (drv_ded.c: wait = nbmes + nbtra), so these
         * numbers must reflect the real queues or nothing is delivered.
         */
        if (chan == 0)
            hm_msg(pty, chan, 1, "%d 0", q_count(0, 'A'));
        else
            hm_msg(pty, chan, 1, "%d %d 0 0 0 %d",
                   q_count(chan, 'S'), q_count(chan, 'I'),
                   ch[chan].used ? ch[chan].st : 0);
        return;
    case '@':
        if (cmd[1] == 'B' || cmd[1] == 'b') { hm_msg(pty, chan, 1, "400"); return; }
        hm_ok(pty, chan);
        return;
    default:
        /* parameter commands (Y/T/O/F/P/W/X/M/K/U/...) -- accept silently */
        hm_ok(pty, chan);
        return;
    }
}

static void handle_hostmode_data(int agwfd, int pty, uint8_t *buf, ssize_t n) {
    for (ssize_t i = 0; i < n; i++) {
        uint8_t c = buf[i];
        switch (hm_st) {
        case 0: hm_chan = c; hm_st = 1; break;
        case 1: hm_flag = c; hm_st = 2; break;     /* 1 = command, 0 = info */
        case 2: hm_need = (size_t)c + 1; hm_got = 0; hm_st = 3; break;
        case 3:
            hm_buf[hm_got++] = c;
            if (hm_got == hm_need) {
                hm_st = 0;
                if (hm_flag) {                     /* command frame */
                    hm_buf[hm_got] = 0;
                    while (hm_got && (hm_buf[hm_got - 1] == '\r' || hm_buf[hm_got - 1] == '\n'))
                        hm_buf[--hm_got] = 0;
                    hm_command(agwfd, pty, hm_chan, (char *)hm_buf);
                } else {                           /* info (data) frame */
                    if (hm_chan > 0 && hm_chan < MAX_CH && ch[hm_chan].used) {
                        agw_send(agwfd, ch[hm_chan].port, 'D', 0xF0, ch[hm_chan].my, ch[hm_chan].peer, hm_buf, (int32_t)hm_got);
                        hm_ok(pty, hm_chan);       /* every host frame gets one reply */
                    } else hm_msg(pty, hm_chan, 2, "CHANNEL NOT CONNECTED");
                }
            }
            break;
        }
    }
}

static void parse_command(int agwfd, int pty, char *line) {
    normalize_call(line);
    /* Belt and braces: strip any control bytes that slipped through. */
    while (*line && (unsigned char)*line < ' ') line++;
    if (!*line) return;
    logv("HOST => %s", line);

    /*
     * FBB's DED driver sends ESC-prefixed JHOST1 to switch the TNC into
     * WA8DED hostmode (drv_ded.c: "\030\033JHOST\r\033MN\r" probe, then
     * "\033JHOST1\r").  Real TF firmware acknowledges JHOST1 silently --
     * rspsuc() is a no-op while still in terminal mode -- and the host
     * immediately begins binary polling.  So: flip modes, say nothing,
     * and answer the polls.  Bare JHOST is a parameter query FBB uses as
     * a reset probe; answer like TF's rsppar would.
     */
    if (!strcasecmp(line, "JHOST1") || !strcasecmp(line, "HOST") || !strcasecmp(line, "HOST1")) {
        hostmode = 1;
        hm_st = 0;
        logv("entering WA8DED hostmode");
        return;
    }
    if (!strcasecmp(line, "JHOST") || !strcasecmp(line, "JHOST0")) {
        host_line(pty, 0, "* 0 *");
        return;
    }

    char cmd = line[0];
    if (cmd >= 'a' && cmd <= 'z') cmd -= 32;
    char *args = line + 1;
    while (*args == ' ' || *args == '\t') args++;

    switch (cmd) {
    case 'I':
        if (*args) {
            strncpy(mycall, args, sizeof(mycall)-1);
            mycall[sizeof(mycall)-1] = 0;
            normalize_call(mycall);
            if (*mycall) agw_send(agwfd, 0, 'X', 0, mycall, "", NULL, 0);
        }
        host_line(pty, 0, "%s", *mycall ? mycall : "NO CALL");
        break;
    case 'S': {                       /* select current channel */
        int v = atoi(args);
        if (v >= 0 && v < MAX_CH) { t_cur = v; host_line(pty, 0, "CHANNEL %d", v); }
        else host_line(pty, 0, "INVALID CHANNEL NUMBER");
        break;
    }
    case 'E':                          /* echo on/off */
        t_echo = (*args != '0');
        break;
    case 'G':
        agw_send(agwfd, 0, 'G', 0, "", "", NULL, 0);
        host_line(pty, 0, "*** Requested AGWPE port information");
        break;
    case 'C': {
        int chan = first_free_channel();
        int port = 0;
        char call[MAX_CALL] = "";
        char via[256] = "";
        if (chan < 0) { host_line(pty, 0, "TNC BUSY - LINE IGNORED"); break; }
        // Accept either "C CALL" or "C PORT CALL [VIA...]".
        char a[64] = "", b[64] = "";
        sscanf(args, "%63s %63s %255[^\r\n]", a, b, via);
        if (a[0] && b[0] && a[0] >= '0' && a[0] <= '9') { port = atoi(a); strncpy(call, b, sizeof(call)-1); }
        else { strncpy(call, a, sizeof(call)-1); }
        normalize_call(call);
        ch[chan].used = true;
        ch[chan].st = 1;                  /* link setup */
        ch[chan].port = port;
        strncpy(ch[chan].my, mycall, sizeof(ch[chan].my)-1);
        strncpy(ch[chan].peer, call, sizeof(ch[chan].peer)-1);
        t_cur = chan;                 /* converse goes here once connected */
        agw_send_connect(agwfd, port, mycall, call, via);
        host_line(pty, chan, "C Command -- Connect Requested -- %s", call);
        break;
    }
    case 'D': {
        int chan = *args ? atoi(args) : t_cur;
        if (chan <= 0 || chan >= MAX_CH || !ch[chan].used) { host_line(pty, 0, "CHANNEL NOT CONNECTED"); break; }
        agw_send(agwfd, ch[chan].port, 'd', 0, ch[chan].my, ch[chan].peer, NULL, 0);
        ch[chan].used = false;
        host_line(pty, chan, "DISCONNECTED fm %s", mycall);
        break;
    }
    case 'M': case 'O': case 'T': case 'Y': case 'F':
    case 'P': case 'W': case 'X': case 'K': case 'N':
    case 'A': case 'B': case 'Z': case 'U': case '@':
        /* parameter commands: accept silently like TF's rspsuc() */
        break;
    default:
        host_line(pty, 0, "INVALID COMMAND: %c", cmd);
        break;
    }
}

/* terminal-mode converse: plain (non-ESC) lines are data for t_cur */
static void terminal_data(int agwfd, int pty, char *line, size_t len) {
    if (t_cur > 0 && t_cur < MAX_CH && ch[t_cur].used) {
        char tmp[MAX_BUF + 1];
        memcpy(tmp, line, len);
        tmp[len] = '\r';
        agw_send(agwfd, ch[t_cur].port, 'D', 0xF0, ch[t_cur].my, ch[t_cur].peer, tmp, (int32_t)(len + 1));
    } else {
        /* No connected channel: try the line as a command anyway, so   */
        /* hosts that omit the ESC prefix (and the old prototype usage) */
        /* keep working.  Real TF would send UI on channel 0 instead.   */
        line[len] = 0;
        parse_command(agwfd, pty, line);
    }
}

static void handle_host_data(int agwfd, int pty, uint8_t *buf, ssize_t n) {
    if (hostmode) { handle_hostmode_data(agwfd, pty, buf, n); return; }

    /*
     * Terminal-mode line discipline, after TF (tfa.c/tfd.c hpinch):
     * ESC as the first character starts a command line and echoes "* ";
     * a plain line is converse data; BS/DEL delete (deleting the ESC
     * erases the prompt); ^U/^X kill the line -- FBB leads its init
     * with ^X for exactly that reason.  Control characters echo as '.'.
     */
    for (ssize_t i = 0; i < n; i++) {
        uint8_t c = buf[i];
        switch (c) {
        case '\r':
        case '\n':
            if (t_echo) write_all(pty, "\r\n", 2);
            t_line[t_lp] = 0;
            if (t_esc) parse_command(agwfd, pty, t_line);
            else if (t_lp) terminal_data(agwfd, pty, t_line, t_lp);
            t_lp = 0;
            t_esc = false;
            break;
        case 0x1b:                          /* ESC: command-line prefix */
            if (t_lp == 0 && !t_esc) {
                t_esc = true;
                if (t_echo) write_all(pty, "* ", 2);
            }
            break;                          /* mid-line ESC ignored */
        case 0x08: case 0x7f:               /* BS / DEL */
            if (t_lp) {
                t_lp--;
                if (t_echo) write_all(pty, "\b \b", 3);
            } else if (t_esc) {
                t_esc = false;
                if (t_echo) write_all(pty, "\b\b  \b\b", 6);
            }
            break;
        case 0x15: case 0x18:               /* ^U / ^X: kill line */
            if (t_echo) {
                while (t_lp) { write_all(pty, "\b \b", 3); t_lp--; }
                if (t_esc) write_all(pty, "\b\b  \b\b", 6);
            }
            t_lp = 0;
            t_esc = false;
            break;
        default:
            if (t_lp + 1 < sizeof(t_line)) {
                if (t_echo) {
                    if (c >= ' ' || c == 7 || c == '\t') write_all(pty, &c, 1);
                    else write_all(pty, ".", 1);
                }
                t_line[t_lp++] = (char)c;
            }
            break;
        }
    }
}

static void unpack_call(char out[MAX_CALL], const char in[10]) {
    int j = 0;
    for (int i = 0; i < 10 && in[i] && j < MAX_CALL-1; i++) out[j++] = in[i];
    out[j] = 0;
    normalize_call(out);
}

static void handle_agw_frame(int pty, const struct agw_hdr *h, const uint8_t *payload) {
    char from[MAX_CALL], to[MAX_CALL];
    unpack_call(from, h->from);
    unpack_call(to, h->to);
    logv("AGW recv kind=%c port=%d from=%s to=%s len=%d", h->kind, h->port, from, to, h->len);

    switch (h->kind) {
    case 'C': { // connected indication
        int ci = find_channel(to, from, h->port);
        if (ci < 0) {
            ci = first_free_channel();
            if (ci > 0) {
                ch[ci].used = true; ch[ci].port = h->port;
                strncpy(ch[ci].my, to, sizeof(ch[ci].my)-1);
                strncpy(ch[ci].peer, from, sizeof(ch[ci].peer)-1);
            }
        }
        if (ci > 0) {
            ch[ci].st = 4;                   /* information transfer */
            if (hostmode) q_status(ci, "(%d) CONNECTED to %s", ci, from);
            else host_line(pty, ci, "CONNECTED With %s", from);
        }
        break;
    }
    case 'd': { // disconnected indication
        int ci = find_channel(to, from, h->port);
        if (ci < 0) ci = find_channel(from, to, h->port);
        if (ci > 0) {
            if (hostmode) q_status(ci, "(%d) DISCONNECTED fm %s", ci, from[0] ? from : to);
            else host_line(pty, ci, "DISCONNECTED From %s", from[0] ? from : to);
            ch[ci].used = false;
            ch[ci].st = 0;
        } else if (!hostmode) host_line(pty, 0, "DISCONNECTED From %s", from[0] ? from : to);
        break;
    }
    case 'D': { // connected data
        int ci = find_channel(to, from, h->port);
        if (ci < 0) ci = find_channel(from, to, h->port);
        if (ci > 0) {
            if (hostmode) {
                if (h->len > 0) q_data(ci, payload, (size_t)h->len);
            } else {
                uint8_t c = (uint8_t)ci;
                write_all(pty, &c, 1);
                if (h->len > 0) write_all(pty, payload, h->len);
                write_all(pty, "\r", 1);
            }
        }
        break;
    }
    case 'G':
        if (hostmode) break;                 /* informational only */
        host_line(pty, 0, "*** Available ports at AGWPE:");
        if (h->len > 0) write_all(pty, payload, h->len);
        write_all(pty, "\r", 1);
        break;
    case 'R':
        if (!hostmode) host_line(pty, 0, "*** AGWPE Version");
        break;
    case 'X':
        if (!hostmode) host_line(pty, 0, "*** Registered with AGWPE the callsign %s", from[0] ? from : mycall);
        break;
    case 'K':
        if (hostmode) break;                 /* no monitor path yet */
        host_line(pty, 0, "Received Frame");
        if (h->len > 0) write_all(pty, payload, h->len);
        write_all(pty, "\r", 1);
        break;
    default:
        break;
    }
}

static void handle_agw_data(int agwfd, int pty) {
    static uint8_t buf[65536];
    static size_t blen = 0;
    ssize_t n = read(agwfd, buf + blen, sizeof(buf) - blen);
    if (n <= 0) return;
    blen += (size_t)n;
    while (blen >= AGW_HDR_LEN) {
        struct agw_hdr h;
        memcpy(&h, buf, AGW_HDR_LEN);
        if (h.len < 0 || h.len > 60000) { blen = 0; return; }
        if (blen < AGW_HDR_LEN + (size_t)h.len) return;
        handle_agw_frame(pty, &h, buf + AGW_HDR_LEN);
        memmove(buf, buf + AGW_HDR_LEN + h.len, blen - AGW_HDR_LEN - h.len);
        blen -= AGW_HDR_LEN + h.len;
    }
}

int main(int argc, char **argv) {
    const char *agw_host = DEFAULT_AGW_HOST;
    const char *agw_port = DEFAULT_AGW_PORT;
    const char *symlink_path = DEFAULT_SYMLINK;
    int opt;
    while ((opt = getopt(argc, argv, "a:p:s:c:vh")) != -1) {
        switch (opt) {
        case 'a': agw_host = optarg; break;
        case 'p': agw_port = optarg; break;
        case 's': symlink_path = optarg; break;
        case 'c': strncpy(mycall, optarg, sizeof(mycall)-1); normalize_call(mycall); break;
        case 'v': verbose++; break;
        default:
            fprintf(stderr, "usage: %s [-a agw_host] [-p agw_port] [-s /tmp/tf2agw] [-c MYCALL] [-v]\n", argv[0]);
            return 2;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int master, slave;
    char slavename[128];
    struct termios tio;
    if (openpty(&master, &slave, slavename, NULL, NULL) < 0) die("openpty: %s", strerror(errno));
    close(slave);
    tcgetattr(master, &tio);
    cfmakeraw(&tio);
    tcsetattr(master, TCSANOW, &tio);
    unlink(symlink_path);
    if (symlink(slavename, symlink_path) < 0) die("symlink %s -> %s: %s", symlink_path, slavename, strerror(errno));
    set_nonblock(master);

    int agwfd = connect_tcp(agw_host, agw_port);
    fprintf(stderr, "tf2agw: PTY %s -> %s, AGW %s:%s\n", symlink_path, slavename, agw_host, agw_port);

    agw_send(agwfd, 0, 'R', 0, "", "", NULL, 0);
    agw_send(agwfd, 0, 'G', 0, "", "", NULL, 0);
    if (*mycall) agw_send(agwfd, 0, 'X', 0, mycall, "", NULL, 0);
    host_line(master, 0, "*** tf2agw linked with AGWPE at %s:%s", agw_host, agw_port);

    while (!stop_flag) {
        struct pollfd pfds[2] = {{master, POLLIN, 0}, {agwfd, POLLIN, 0}};
        int r = poll(pfds, 2, 500);
        if (r < 0) { if (errno == EINTR) continue; die("poll: %s", strerror(errno)); }
        if (pfds[0].revents & POLLIN) {
            uint8_t b[1024]; ssize_t n = read(master, b, sizeof(b));
            if (n > 0) handle_host_data(agwfd, master, b, n);
        }
        if (pfds[1].revents & POLLIN) handle_agw_data(agwfd, master);
        if (pfds[1].revents & (POLLERR|POLLHUP|POLLNVAL)) break;
    }
    close(agwfd); close(master); unlink(symlink_path);
    return 0;
}
