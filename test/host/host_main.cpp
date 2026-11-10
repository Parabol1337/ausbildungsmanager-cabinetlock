// Runs the REAL, unmodified RP2040-ETH_Cabinet.ino (and Modbus.ino) on a PC. The runner plays the hardware around it:
//  - the CH9120 network bridge: a TCP connection to the server, delivered to the sketch as serial data on Serial2,
//    with the "not connected" pin (D17) following the connection (and reconnecting like the chip does)
//  - the RS485 bus with two Modbus relay modules (addresses 1 and 2): answers input requests with the doors that are
//    closed or open, takes relay commands (and answers them like the real module does), and, for address 0, a single
//    module whose address can be set and read
//
//   host_test <server-host> <server-port>
// stdin: open <channel> | closed <channel> | silent | talk | button | release | quit
// stdout: READY, RELAY <channel> <flash>, MODBUS-BAD (a frame with a wrong CRC arrived)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "Arduino.h"
#include "Stream.h"
#include "EEPROM.h"
#include "stubs/CH9120.h"

int g_pinLevel[64];
uint32_t g_millis = 0;
bool BOOTSEL = false;
EEPROMClass EEPROM;
void CH9120_init(void) {}

namespace {
int g_fd = -1;
const char* g_host;
int g_port;
uint32_t g_retryAt = 0;

uint32_t nowMs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint32_t(ts.tv_sec) * 1000u + uint32_t(ts.tv_nsec / 1000000);
}

struct StdoutSerial : HardwareSerial {
    size_t write(uint8_t b) override { fputc(b, stdout); if (b == '\n') fflush(stdout); return 1; }
};

// UART2 as the sketch sees it: bytes from and to the TCP connection held by the "chip".
struct BridgeSerial : HardwareSerial {
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t* data, size_t length) override {
        if (g_fd < 0) return 0;
        ssize_t n = send(g_fd, data, length, MSG_NOSIGNAL | MSG_DONTWAIT);
        return n > 0 ? size_t(n) : 0;
    }
    int available() override {
        if (g_fd < 0) return 0;
        int n = 0;
        ioctl_available(n);
        return n;
    }
    int read() override {
        uint8_t b;
        if (g_fd < 0) return -1;
        ssize_t n = recv(g_fd, &b, 1, MSG_DONTWAIT);
        return n == 1 ? b : -1;
    }
    int peek() override { return -1; }
    void ioctl_available(int& n);
};

StdoutSerial g_stdout;
BridgeSerial g_bridge;
}  // namespace

HardwareSerial& Serial = g_stdout;
HardwareSerial& Serial1 = g_stdout;
HardwareSerial& Serial2 = g_bridge;

#include <sys/ioctl.h>
void BridgeSerial::ioctl_available(int& n) { ::ioctl(g_fd, FIONREAD, &n); }

// The chip: connect to the server, report the state on the "not connected" pin, reconnect when the link is lost.
static void bridgeStep(uint32_t now) {
    if (g_fd >= 0) {
        pollfd p = {g_fd, POLLIN, 0};
        if (poll(&p, 1, 0) > 0 && (p.revents & (POLLIN | POLLHUP | POLLERR))) {
            char c;
            if (recv(g_fd, &c, 1, MSG_PEEK | MSG_DONTWAIT) == 0) {   // server closed
                close(g_fd);
                g_fd = -1;
                g_retryAt = now + 300;
            }
        }
    } else if (static_cast<int32_t>(now - g_retryAt) >= 0) {
        addrinfo hints = {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* res = nullptr;
        char port[8];
        snprintf(port, sizeof(port), "%d", g_port);
        if (getaddrinfo(g_host, port, &hints, &res) == 0) {
            int fd = socket(res->ai_family, res->ai_socktype, 0);
            if (fd >= 0 && connect(fd, res->ai_addr, res->ai_addrlen) == 0) {
                int one = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
                g_fd = fd;
            } else if (fd >= 0) {
                close(fd);
            }
            freeaddrinfo(res);
        }
        if (g_fd < 0) g_retryAt = now + 300;
    }
    g_pinLevel[D17] = g_fd >= 0 ? LOW : HIGH;   // the CH9120 pin is HIGH while not connected
}


// ---- RS485 bus with the relay modules ---------------------------------------------------------------------------------
#define Modbus_Serial g_modbus

#include <deque>
#include <vector>

namespace {
uint8_t g_inputs[256];          // by module address: bit n-1 = 1 means input n is "door closed"
uint8_t g_moduleAddress = 255;  // address of a module that is programmed (the only one on the bus)
bool g_silent = false;

uint16_t crcOf(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int k = 0; k < 8; k++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}
}  // namespace

struct ModbusBus : HardwareSerial {
    using Print::write;
    std::vector<uint8_t> out;     // what the sketch sent
    std::deque<uint8_t> in;       // what the modules answered
    size_t write(uint8_t b) override { out.push_back(b); return 1; }
    int available() override { return int(in.size()); }
    int read() override { if (in.empty()) return -1; int b = in.front(); in.pop_front(); return b; }
    int peek() override { return in.empty() ? -1 : in.front(); }
    void flush() override { process(); }
    void reply(std::vector<uint8_t> frame) {
        uint16_t crc = crcOf(frame.data(), frame.size());
        frame.push_back(crc & 0xff);
        frame.push_back(crc >> 8);
        for (uint8_t b : frame) in.push_back(b);
    }
    void process();
};
ModbusBus g_modbus;

// ---- the real sketch -----------------------------------------------------------------------------------------------
#include "../../RP2040-ETH_Cabinet.ino"
// ---------------------------------------------------------------------------------------------------------------------

// The bus: two modules (1 and 2) answer for themselves; address 0 reaches a module that is alone on the bus.
void ModbusBus::process() {
    if (out.size() < 8) { out.clear(); return; }
    uint16_t crc = crcOf(out.data(), out.size() - 2);
    if (out[out.size() - 2] != (crc & 0xff) || out.back() != (crc >> 8)) { ::printf("MODBUS-BAD\n"); fflush(stdout); out.clear(); return; }
    uint8_t slave = out[0];
    if (g_silent) { out.clear(); return; }
    if (slave == 0 && out[1] == 0x10) {                       // set the address of the module on the bus
        g_moduleAddress = out[8];
        ::printf("ADDRESS %d\n", g_moduleAddress);
        fflush(stdout);
        reply({0x00, 0x10, 0x00, 0x00, 0x00, 0x01});
    } else if (slave == 0 && out[1] == 0x03) {                // read it
        reply({0x00, 0x03, 0x02, 0x00, g_moduleAddress});
    } else if (slave >= 1 && slave <= 2) {
        if (out[1] == 0x02) {
            reply({slave, 0x02, 0x01, g_inputs[slave]});
        } else if (out[1] == 0x10) {
            int reg = out[2] * 256 + out[3];
            int relay = (reg - 3) / 5 + 1;
            int ch = 0;
            for (unsigned i = 0; i < Channel; i++) if (Cabinets[i].module == slave && Cabinets[i].relay == relay) ch = int(i) + 1;
            ::printf("RELAY %d %d\n", ch, out[10]);
            fflush(stdout);
            reply({slave, 0x10, out[2], out[3], 0x00, 0x02});
        }
    }
    out.clear();
}

static void command(char* line) {
    char* arg = strchr(line, ' ');
    if (arg) *arg++ = 0;
    if ((!strcmp(line, "open") || !strcmp(line, "closed")) && arg) {
        int ch = atoi(arg) - 1;
        if (ch >= 0 && ch < (int)Channel && Cabinets[ch].contact) {
            uint8_t bit = uint8_t(1u << (Cabinets[ch].contact - 1));
            if (!strcmp(line, "open")) g_inputs[Cabinets[ch].module] &= uint8_t(~bit);
            else g_inputs[Cabinets[ch].module] |= bit;
        }
    } else if (!strcmp(line, "silent")) {
        g_silent = true;
    } else if (!strcmp(line, "talk")) {
        g_silent = false;
    } else if (!strcmp(line, "button")) {
        BOOTSEL = true;
    } else if (!strcmp(line, "release")) {
        BOOTSEL = false;
    } else if (!strcmp(line, "quit")) {
        printf("BYE\n");
        fflush(stdout);
        exit(0);
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s host port\n", argv[0]);
        return 2;
    }
    for (int fd = 3; fd < 256; fd++) close(fd);   // sockets inherited from the test harness
    g_host = argv[1];
    g_port = atoi(argv[2]);
    g_pinLevel[D17] = HIGH;
    memset(g_inputs, 0xff, sizeof(g_inputs));   // all doors closed

    g_millis = nowMs();
    setup();
    printf("READY\n");
    fflush(stdout);

    char buf[256];
    size_t used = 0;
    for (;;) {
        g_millis = nowMs();
        bridgeStep(g_millis);
        loop();
        pollfd p = {0, POLLIN, 0};
        if (poll(&p, 1, 2) > 0 && (p.revents & POLLIN)) {
            ssize_t n = read(0, buf + used, sizeof(buf) - used - 1);
            if (n == 0) return 0;
            if (n > 0) {
                used += size_t(n);
                buf[used] = 0;
                for (;;) {
                    char* nl = strchr(buf, '\n');
                    if (!nl) break;
                    *nl = 0;
                    command(buf);
                    size_t consumed = size_t(nl + 1 - buf);
                    memmove(buf, buf + consumed, used - consumed + 1);
                    used -= consumed;
                }
            }
        }
    }
}
