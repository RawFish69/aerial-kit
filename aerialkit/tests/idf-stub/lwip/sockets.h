#ifndef AK_HOST_IDF_LWIP_SOCKETS_H
#define AK_HOST_IDF_LWIP_SOCKETS_H

/*
 * lwip's socket API, as the net port uses it: one listening socket, one client
 * at a time, `select` to wait on it, `recv` to take what arrived and `send` to
 * push what the loop produced.
 *
 * IDF's lwip maps these names onto `lwip_*` with macros; the stand-in declares
 * the names directly, because the port calls the names and nothing here
 * pretends to be the lwip it would be calling on the chip. The shapes are
 * lwip's own (components/lwip/include/lwip/sockets.h and inet.h), including
 * that `struct sockaddr_in` holds a network-order address and port, so the
 * model can check that the port bound the port it says it bound.
 */

#include <stdint.h>
#include <sys/types.h> /* ssize_t, which is what lwip's recv and send return */

#define AF_INET     2
#define SOCK_STREAM 1
#define IPPROTO_IP  0

#define SOL_SOCKET   0xFFF
#define SO_REUSEADDR 0x0004

#define INADDR_ANY 0x00000000u

typedef uint32_t in_addr_t;
typedef uint16_t in_port_t;
typedef uint32_t socklen_t;

struct sockaddr;

struct in_addr {
    in_addr_t s_addr;
};

struct sockaddr_in {
    uint8_t        sin_len;
    uint8_t        sin_family;
    in_port_t      sin_port;
    struct in_addr sin_addr;
    char           sin_zero[8];
};

struct timeval {
    long tv_sec;
    long tv_usec;
};

/* lwip's fd_set is a bitset over a small range of descriptors; the port uses
 * exactly one bit of it. */
typedef struct {
    unsigned long bits[8];
} fd_set;

#define FD_ZERO(set)     do { (set)->bits[0] = 0ul; } while (0)
#define FD_SET(fd, set)  do { (set)->bits[0] |= 1ul << (fd); } while (0)

/* The model's own names, reached through lwip's names.
 *
 * IDF reaches lwip by renaming these to `lwip_*`; this stand-in renames them
 * to the model's, and for a second reason as well: `close`, `send`, `recv`,
 * `select`, `socket`, `bind` and `htons` are the C library's names too, and a
 * model that defined those in the test binary would interpose the library's
 * for the whole process - `close` is how stdio shuts a stream down. The port
 * cannot tell the difference, which is the point. */
int ak_host_net_socket(int domain, int type, int protocol);
int ak_host_net_setsockopt(int fd, int level, int option, const void *value,
                           socklen_t length);
int ak_host_net_bind(int fd, const struct sockaddr *address, socklen_t length);
int ak_host_net_listen(int fd, int backlog);
int ak_host_net_accept(int fd, struct sockaddr *address, socklen_t *length);
int ak_host_net_select(int nfds, fd_set *readable, fd_set *writable,
                       fd_set *except, struct timeval *timeout);
ssize_t ak_host_net_recv(int fd, void *buffer, size_t length, int flags);
ssize_t ak_host_net_send(int fd, const void *buffer, size_t length, int flags);
int ak_host_net_close(int fd);

uint32_t ak_host_net_htonl(uint32_t host);
uint16_t ak_host_net_htons(uint16_t host);

#define socket(d, t, p)        ak_host_net_socket(d, t, p)
#define setsockopt(f, l, o, v, n) ak_host_net_setsockopt(f, l, o, v, n)
#define bind(f, a, n)          ak_host_net_bind(f, a, n)
#define listen(f, b)           ak_host_net_listen(f, b)
#define accept(f, a, n)        ak_host_net_accept(f, a, n)
#define select(n, r, w, e, t)  ak_host_net_select(n, r, w, e, t)
#define recv(f, b, n, g)       ak_host_net_recv(f, b, n, g)
#define send(f, b, n, g)       ak_host_net_send(f, b, n, g)
#define close(f)               ak_host_net_close(f)
#define htonl(h)               ak_host_net_htonl(h)
#define htons(h)               ak_host_net_htons(h)

#endif /* AK_HOST_IDF_LWIP_SOCKETS_H */
