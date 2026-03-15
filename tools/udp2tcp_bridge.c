/*
 * UDP-to-TCP Bridge for RTL-SDR WiFi Bridge
 *
 * Receives IQ datagrams from the ESP32-P4 UDP server, strips the
 * 8-byte header (seq + timestamp), and relays raw IQ data to
 * rtl_tcp clients over TCP. Provides the DongleInfo header on connect.
 *
 * Usage: udp2tcp_bridge [options]
 *   -u <host>:<port>   UDP source (default: 192.168.1.232:1235)
 *   -t <port>          TCP listen port (default: 1234)
 *   -s                 Print statistics every second
 *
 * Connect SDR++ to localhost:1234, bridge receives UDP from ESP32-P4
 * and forwards as standard rtl_tcp stream.
 *
 * Build: gcc -O2 -o udp2tcp_bridge udp2tcp_bridge.c -lpthread
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/time.h>

/* RTL-TCP DongleInfo header */
typedef struct __attribute__((packed)) {
    uint32_t magic;         /* "RTL0" = 0x52544C30 */
    uint32_t tuner_type;    /* 6 = R828D */
    uint32_t gain_count;    /* 29 */
} dongle_info_t;

/* UDP packet header from ESP32-P4 */
typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t timestamp_us;
} udp_header_t;

/* RTL-TCP command (5 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  cmd;
    uint32_t param;
} rtltcp_cmd_t;

#define UDP_HEADER_SIZE     8
#define MAX_UDP_PACKET      2048
#define DEFAULT_UDP_HOST    "192.168.1.232"
#define DEFAULT_UDP_PORT    1235
#define DEFAULT_TCP_PORT    1234
#define TUNER_R828D         6
#define GAIN_COUNT          29

static volatile int running = 1;
static int show_stats = 0;

/* Statistics */
static uint64_t total_udp_packets = 0;
static uint64_t total_udp_bytes = 0;
static uint64_t total_tcp_bytes = 0;
static uint64_t dropped_packets = 0;
static uint32_t last_seq = 0;
static int tcp_client_fd = -1;

/* UDP source */
static char udp_host[64] = DEFAULT_UDP_HOST;
static int  udp_port = DEFAULT_UDP_PORT;
static int  tcp_port = DEFAULT_TCP_PORT;

/* UDP socket (also used for sending commands) */
static int udp_fd = -1;
static struct sockaddr_in udp_dest;

static void sighandler(int sig) {
    (void)sig;
    running = 0;
}

/* Forward rtl_tcp commands from TCP client to UDP */
static void *cmd_thread(void *arg) {
    (void)arg;
    uint8_t buf[5];

    while (running && tcp_client_fd >= 0) {
        int n = recv(tcp_client_fd, buf, 5, MSG_WAITALL);
        if (n == 5 && udp_fd >= 0) {
            /* Forward command to ESP32-P4 via UDP */
            sendto(udp_fd, buf, 5, 0,
                   (struct sockaddr *)&udp_dest, sizeof(udp_dest));
            printf("CMD: 0x%02x param=%u\n", buf[0],
                   (buf[1]<<24)|(buf[2]<<16)|(buf[3]<<8)|buf[4]);
        } else if (n <= 0) {
            break;
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    int opt;
    while ((opt = getopt(argc, argv, "u:t:sh")) != -1) {
        switch (opt) {
        case 'u': {
            char *colon = strchr(optarg, ':');
            if (colon) {
                *colon = 0;
                strncpy(udp_host, optarg, sizeof(udp_host)-1);
                udp_port = atoi(colon+1);
            } else {
                strncpy(udp_host, optarg, sizeof(udp_host)-1);
            }
            break;
        }
        case 't': tcp_port = atoi(optarg); break;
        case 's': show_stats = 1; break;
        case 'h':
        default:
            fprintf(stderr,
                "Usage: %s [-u host:port] [-t tcp_port] [-s]\n"
                "  -u  UDP source (default %s:%d)\n"
                "  -t  TCP listen port (default %d)\n"
                "  -s  Show statistics\n",
                argv[0], DEFAULT_UDP_HOST, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
            return 1;
        }
    }

    signal(SIGINT, sighandler);
    signal(SIGPIPE, SIG_IGN);

    /* Create UDP socket for receiving IQ and sending commands */
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) { perror("udp socket"); return 1; }

    struct sockaddr_in udp_bind = {
        .sin_family = AF_INET,
        .sin_port = 0,  /* any port */
        .sin_addr.s_addr = INADDR_ANY,
    };
    bind(udp_fd, (struct sockaddr *)&udp_bind, sizeof(udp_bind));

    /* Set recv timeout */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(udp_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* Increase receive buffer */
    int rcvbuf = 1024 * 1024;
    setsockopt(udp_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    /* ESP32-P4 UDP destination */
    udp_dest.sin_family = AF_INET;
    udp_dest.sin_port = htons(udp_port);
    inet_aton(udp_host, &udp_dest.sin_addr);

    /* Subscribe to UDP stream by sending a ping */
    uint8_t ping = 0x00;
    sendto(udp_fd, &ping, 1, 0,
           (struct sockaddr *)&udp_dest, sizeof(udp_dest));
    printf("Subscribed to UDP stream at %s:%d\n", udp_host, udp_port);

    /* Create TCP listening socket */
    int tcp_listen = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_listen < 0) { perror("tcp socket"); return 1; }

    int reuse = 1;
    setsockopt(tcp_listen, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in tcp_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(tcp_port),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(tcp_listen, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0) {
        perror("tcp bind"); return 1;
    }
    listen(tcp_listen, 1);

    printf("RTL-TCP bridge listening on TCP port %d\n", tcp_port);
    printf("Connect SDR++ to localhost:%d (RTL-TCP source)\n", tcp_port);

    uint8_t pkt_buf[MAX_UDP_PACKET];
    struct timeval stats_time;
    gettimeofday(&stats_time, NULL);
    uint64_t stats_bytes = 0;

    while (running) {
        /* Accept TCP client (non-blocking check) */
        if (tcp_client_fd < 0) {
            struct timeval accept_tv = { .tv_sec = 0, .tv_usec = 100000 };
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(tcp_listen, &fds);
            if (select(tcp_listen + 1, &fds, NULL, NULL, &accept_tv) > 0) {
                tcp_client_fd = accept(tcp_listen, NULL, NULL);
                if (tcp_client_fd >= 0) {
                    int nodelay = 0;
                    setsockopt(tcp_client_fd, IPPROTO_TCP, TCP_NODELAY,
                               &nodelay, sizeof(nodelay));
                    int sndbuf = 262144;
                    setsockopt(tcp_client_fd, SOL_SOCKET, SO_SNDBUF,
                               &sndbuf, sizeof(sndbuf));

                    /* Send DongleInfo header */
                    dongle_info_t info = {
                        .magic = htonl(0x52544C30),
                        .tuner_type = htonl(TUNER_R828D),
                        .gain_count = htonl(GAIN_COUNT),
                    };
                    send(tcp_client_fd, &info, sizeof(info), 0);
                    printf("TCP client connected\n");

                    /* Re-subscribe UDP */
                    sendto(udp_fd, &ping, 1, 0,
                           (struct sockaddr *)&udp_dest, sizeof(udp_dest));

                    /* Start command forwarding thread */
                    pthread_t cmd_tid;
                    pthread_create(&cmd_tid, NULL, cmd_thread, NULL);
                    pthread_detach(cmd_tid);
                }
            }
        }

        /* Receive UDP packet */
        int n = recvfrom(udp_fd, pkt_buf, sizeof(pkt_buf), 0, NULL, NULL);
        if (n <= 0) continue;
        if (n <= UDP_HEADER_SIZE) continue;

        total_udp_packets++;
        total_udp_bytes += n;

        /* Parse header */
        udp_header_t *hdr = (udp_header_t *)pkt_buf;
        uint32_t seq = hdr->seq;

        /* Check for gaps */
        if (total_udp_packets > 1 && seq != last_seq + 1) {
            uint32_t gap = seq - last_seq - 1;
            dropped_packets += gap;
        }
        last_seq = seq;

        /* Strip header, forward IQ data to TCP client */
        int iq_len = n - UDP_HEADER_SIZE;
        uint8_t *iq_data = pkt_buf + UDP_HEADER_SIZE;

        if (tcp_client_fd >= 0) {
            int sent = send(tcp_client_fd, iq_data, iq_len, MSG_NOSIGNAL);
            if (sent < 0) {
                if (errno == EPIPE || errno == ECONNRESET) {
                    printf("TCP client disconnected\n");
                    close(tcp_client_fd);
                    tcp_client_fd = -1;
                }
            } else {
                total_tcp_bytes += sent;
            }
        }

        stats_bytes += iq_len;

        /* Print stats every second */
        if (show_stats) {
            struct timeval now;
            gettimeofday(&now, NULL);
            double elapsed = (now.tv_sec - stats_time.tv_sec) +
                             (now.tv_usec - stats_time.tv_usec) / 1e6;
            if (elapsed >= 1.0) {
                double rate_ksps = stats_bytes / 2.0 / elapsed / 1000.0;
                printf("UDP: %.1f kSPS (%.2f Mbps) | pkts=%lu drop=%lu | TCP: %lu KB\n",
                       rate_ksps, stats_bytes * 8.0 / elapsed / 1e6,
                       (unsigned long)total_udp_packets,
                       (unsigned long)dropped_packets,
                       (unsigned long)(total_tcp_bytes / 1024));
                stats_bytes = 0;
                stats_time = now;
            }
        }
    }

    printf("\nShutting down...\n");
    printf("Total: UDP %lu packets (%lu KB), TCP %lu KB, dropped %lu\n",
           (unsigned long)total_udp_packets,
           (unsigned long)(total_udp_bytes / 1024),
           (unsigned long)(total_tcp_bytes / 1024),
           (unsigned long)dropped_packets);

    if (tcp_client_fd >= 0) close(tcp_client_fd);
    close(tcp_listen);
    close(udp_fd);
    return 0;
}
