/*
 * POSIX UDP sockets for host-side communication (inspector + command channel).
 * Kept in a separate TU to avoid FreeRTOS_Sockets.h macro collisions with
 * POSIX struct sockaddr_in.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int inspector_fd = -1;
static struct sockaddr_in inspector_peer;

void host_inspector_open(void) {
    const char *h = getenv("HARNESS_INSPECTOR_HOST");
    const char *p = getenv("HARNESS_INSPECTOR_PORT");
    if (!h || !p) {
        fprintf(stderr,
                "harness: HARNESS_INSPECTOR_{HOST,PORT} not set — inspector "
                "disabled\n");
        return;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("inspector socket");
        return;
    }

    memset(&inspector_peer, 0, sizeof(inspector_peer));
    inspector_peer.sin_family = AF_INET;
    inspector_peer.sin_port = htons((uint16_t)atoi(p));
    inet_pton(AF_INET, h, &inspector_peer.sin_addr);
    inspector_fd = fd;
}

void host_inspector_emit(const char *line) {
    if (inspector_fd < 0) return;
    sendto(inspector_fd, line, strlen(line), 0,
           (struct sockaddr *)&inspector_peer, sizeof(inspector_peer));
}

static int cmd_fd = -1;

int host_cmd_open(int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("cmd socket");
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("cmd bind");
        close(fd);
        return -1;
    }

    cmd_fd = fd;
    return fd;
}

int host_cmd_recv(char *buf, int size) {
    if (cmd_fd < 0) return -1;
    return (int)recvfrom(cmd_fd, buf, size, 0, NULL, NULL);
}
