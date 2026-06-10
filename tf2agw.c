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
#define MAX_CH 10
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
    char my[MAX_CALL];
    char peer[MAX_CALL];
};

static volatile sig_atomic_t stop_flag = 0;
static int verbose = 0;
static struct chan ch[MAX_CH];
static char mycall[MAX_CALL] = "";

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

static void host_line(int pty, int channel, const char *fmt, ...) {
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    unsigned char c = (unsigned char)channel;
    write_all(pty, &c, 1);
    write_all(pty, line, strlen(line));
    write_all(pty, "\r", 1);
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

static void parse_command(int agwfd, int pty, char *line) {
    normalize_call(line);
    if (!*line) return;
    logv("HOST => %s", line);

    /*
     * FBB starts TF/WA8DED style ports by sending HOST or HOST1.
     * This is not an AX.25 connect request; it is the command that puts a
     * real TF/DED TNC into hostmode.  The first prototype treated it as an
     * unknown H command and replied "TNC BUSY", causing FBB to loop.
     */
    if (!strcasecmp(line, "HOST") || !strcasecmp(line, "HOST1")) {
        host_line(pty, 0, "HOSTMODE ON");
        return;
    }

    char cmd = line[0];
    if (cmd >= 'a' && cmd <= 'z') cmd -= 32;
    char *args = line + 1;
    while (*args == ' ' || *args == '\t') args++;

    switch (cmd) {
    case 'J':
        host_line(pty, 0, "JHOST");
        break;
    case 'I':
        strncpy(mycall, args, sizeof(mycall)-1);
        mycall[sizeof(mycall)-1] = 0;
        normalize_call(mycall);
        if (*mycall) agw_send(agwfd, 0, 'X', 0, mycall, "", NULL, 0);
        host_line(pty, 0, "MYCALL %s", mycall);
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
        ch[chan].port = port;
        strncpy(ch[chan].my, mycall, sizeof(ch[chan].my)-1);
        strncpy(ch[chan].peer, call, sizeof(ch[chan].peer)-1);
        agw_send(agwfd, port, 'C', 0, mycall, call, via, strlen(via));
        host_line(pty, chan, "C Command -- Connect Requested -- %s", call);
        break;
    }
    case 'D': {
        int chan = atoi(args);
        if (chan <= 0 || chan >= MAX_CH || !ch[chan].used) { host_line(pty, 0, "CHANNEL NOT CONNECTED"); break; }
        agw_send(agwfd, ch[chan].port, 'd', 0, ch[chan].my, ch[chan].peer, NULL, 0);
        ch[chan].used = false;
        host_line(pty, chan, "DISCONNECTED fm %s", mycall);
        break;
    }
    default:
        host_line(pty, 0, "TNC BUSY - LINE IGNORED");
        break;
    }
}

static void handle_host_data(int agwfd, int pty, uint8_t *buf, ssize_t n) {
    static char line[MAX_BUF];
    static size_t lp = 0;
    static int data_chan = -1;
    static uint8_t data[MAX_BUF];
    static size_t dp = 0;

    for (ssize_t i = 0; i < n; i++) {
        uint8_t c = buf[i];
        // Experimental: channel-prefixed data path. If first byte is 1..9, collect until CR/LF and send as AGW data.
        if (lp == 0 && dp == 0 && c > 0 && c < MAX_CH && ch[c].used) {
            data_chan = c;
            continue;
        }
        if (data_chan > 0) {
            if (c == '\r' || c == '\n') {
                if (dp) agw_send(agwfd, ch[data_chan].port, 'D', 0, ch[data_chan].my, ch[data_chan].peer, data, (int32_t)dp);
                dp = 0; data_chan = -1;
            } else if (dp < sizeof(data)) data[dp++] = c;
            continue;
        }
        if (c == '\r' || c == '\n') {
            line[lp] = 0;
            parse_command(agwfd, pty, line);
            lp = 0;
        } else if (lp + 1 < sizeof(line)) line[lp++] = (char)c;
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
        if (ci > 0) host_line(pty, ci, "CONNECTED With %s", from);
        break;
    }
    case 'd': { // disconnected indication
        int ci = find_channel(to, from, h->port);
        if (ci < 0) ci = find_channel(from, to, h->port);
        if (ci > 0) {
            host_line(pty, ci, "DISCONNECTED From %s", from[0] ? from : to);
            ch[ci].used = false;
        } else host_line(pty, 0, "DISCONNECTED From %s", from[0] ? from : to);
        break;
    }
    case 'D': { // connected data
        int ci = find_channel(to, from, h->port);
        if (ci < 0) ci = find_channel(from, to, h->port);
        if (ci > 0) {
            uint8_t c = (uint8_t)ci;
            write_all(pty, &c, 1);
            if (h->len > 0) write_all(pty, payload, h->len);
            write_all(pty, "\r", 1);
        }
        break;
    }
    case 'G':
        host_line(pty, 0, "*** Available ports at AGWPE:");
        if (h->len > 0) write_all(pty, payload, h->len);
        write_all(pty, "\r", 1);
        break;
    case 'R':
        host_line(pty, 0, "*** AGWPE Version");
        break;
    case 'X':
        host_line(pty, 0, "*** Registered with AGWPE the callsign %s", from[0] ? from : mycall);
        break;
    case 'K':
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
