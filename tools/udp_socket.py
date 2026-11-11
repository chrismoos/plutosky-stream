"""Receive buffer sizing and drop counts for the tools' UDP sockets.

A Python reader on a loaded host can fall behind a burst of datagrams. Once
the socket's receive buffer is full the kernel drops what arrives, and a drop
at the end of a burst leaves no gap in the packet count to show it. The kernel
counts every such drop, so a tool can tell its own lost datagrams apart from
ones the sender never sent.
"""

import os
import socket

# Linux socket option that sets the buffer past net.core.rmem_max. It needs
# CAP_NET_ADMIN, and SO_RCVBUF is the fallback, capped at rmem_max.
LINUX_SO_RCVBUFFORCE = 33
PROC_NET_UDP = "/proc/net/udp"
INODE_COLUMN = 9
DROPS_COLUMN = 12


def enlarge_receive_buffer(sock, size):
    """Asks for a receive buffer of size bytes and returns the size granted."""
    try:
        sock.setsockopt(socket.SOL_SOCKET, LINUX_SO_RCVBUFFORCE, size)
    except OSError:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, size)
    return sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)


def receive_drops(sock):
    """Datagrams the kernel dropped for this IPv4 socket because its receive
    buffer was full, or None where /proc/net/udp is not available."""
    try:
        inode = str(os.fstat(sock.fileno()).st_ino)
        with open(PROC_NET_UDP) as table:
            next(table)
            for line in table:
                fields = line.split()
                if len(fields) > DROPS_COLUMN and fields[INODE_COLUMN] == inode:
                    return int(fields[DROPS_COLUMN])
    except (OSError, ValueError):
        return None
    return None
