// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exercise the /dev/gemslot0 ABI: print the pool geometry, send one
 * Ethernet/IPv4/UDP frame out of a transmit slot, collect its completion, then
 * steer a receive port and print what arrives.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/gemslot.h>

#define DEV		"/dev/gemslot0"
#define ETH_ALEN	6
#define ETH_P_IP	0x0800
#define IPPROTO_UDP_	17
#define IPV4_IHL_WORDS	5
#define IPV4_TTL	64
#define MIN_FRAME_LEN	60
#define DUMP_BYTES	32

struct opts {
	const char *dst_mac;
	const char *src_ip;
	const char *dst_ip;
	unsigned int src_port;
	unsigned int dst_port;
	unsigned int payload;
	unsigned int steer_port;
	unsigned int rx_ms;
};

static int parse_mac(const char *s, unsigned char *out)
{
	unsigned int b[ETH_ALEN];
	int i;

	if (sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4],
		   &b[5]) != ETH_ALEN)
		return -1;
	for (i = 0; i < ETH_ALEN; i++)
		out[i] = (unsigned char)b[i];

	return 0;
}

static unsigned short ip_checksum(const void *buf, unsigned int len)
{
	const unsigned char *p = buf;
	unsigned long sum = 0;
	unsigned int i;

	for (i = 0; i + 1 < len; i += 2)
		sum += ((unsigned long)p[i] << 8) | p[i + 1];
	if (i < len)
		sum += (unsigned long)p[i] << 8;
	while (sum >> 16)
		sum = (sum & 0xffff) + (sum >> 16);

	return (unsigned short)~sum;
}

static void put_be16(unsigned char *p, unsigned int v)
{
	p[0] = (unsigned char)(v >> 8);
	p[1] = (unsigned char)v;
}

/* Returns the frame length without the FCS, padded to MIN_FRAME_LEN. */
static unsigned int build_frame(unsigned char *f, const struct opts *o,
				const unsigned char *src_mac)
{
	unsigned char dst_mac[ETH_ALEN];
	unsigned int ip = 14, udp = ip + IPV4_IHL_WORDS * 4;
	unsigned int len = udp + 8 + o->payload;
	struct in_addr sa, da;
	unsigned int i;

	if (parse_mac(o->dst_mac, dst_mac)) {
		fprintf(stderr, "bad destination MAC %s\n", o->dst_mac);
		exit(1);
	}
	if (!inet_aton(o->src_ip, &sa) || !inet_aton(o->dst_ip, &da)) {
		fprintf(stderr, "bad IP address\n");
		exit(1);
	}

	memset(f, 0, len);
	memcpy(f, dst_mac, ETH_ALEN);
	memcpy(f + ETH_ALEN, src_mac, ETH_ALEN);
	put_be16(f + 12, ETH_P_IP);

	f[ip] = 0x40 | IPV4_IHL_WORDS;
	put_be16(f + ip + 2, len - ip);
	f[ip + 8] = IPV4_TTL;
	f[ip + 9] = IPPROTO_UDP_;
	memcpy(f + ip + 12, &sa.s_addr, 4);
	memcpy(f + ip + 16, &da.s_addr, 4);
	put_be16(f + ip + 10, ip_checksum(f + ip, IPV4_IHL_WORDS * 4));

	put_be16(f + udp, o->src_port);
	put_be16(f + udp + 2, o->dst_port);
	put_be16(f + udp + 4, 8 + o->payload);
	/* UDP checksum stays zero, which IPv4 allows */

	for (i = 0; i < o->payload; i++)
		f[udp + 8 + i] = (unsigned char)i;

	return len < MIN_FRAME_LEN ? MIN_FRAME_LEN : len;
}

static int read_if_mac(const char *ifname, unsigned char *out)
{
	char path[128];
	char buf[32];
	int fd, n;

	snprintf(path, sizeof(path), "/sys/class/net/%s/address", ifname);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;
	n = (int)read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return -1;
	buf[n] = 0;

	return parse_mac(buf, out);
}

static void dump(const unsigned char *p, unsigned int len)
{
	unsigned int i, n = len < DUMP_BYTES ? len : DUMP_BYTES;

	for (i = 0; i < n; i++)
		printf("%02x%s", p[i], i + 1 == n ? "" : " ");
	printf("%s\n", len > n ? " ..." : "");
}

static void usage(void)
{
	fprintf(stderr,
		"usage: gemslot_test [-m dst_mac] [-s src_ip] [-d dst_ip]\n"
		"                    [-p dst_port] [-q src_port] [-n payload]\n"
		"                    [-r steer_port] [-t rx_ms]\n");
	exit(1);
}

int main(int argc, char **argv)
{
	struct opts o = {
		.dst_mac = "ff:ff:ff:ff:ff:ff",
		.src_ip = "192.168.2.1",
		.dst_ip = "192.168.2.10",
		.src_port = 30000,
		.dst_port = 30002,
		.payload = 64,
		.steer_port = 30002,
		.rx_ms = 2000,
	};
	unsigned char src_mac[ETH_ALEN] = { 0 };
	struct gemslot_tx_batch tx = { 0 };
	struct gemslot_rx_batch rx = { 0 };
	struct gemslot_steer steer;
	struct gemslot_wait wait;
	struct gemslot_info info;
	unsigned char *pool;
	unsigned int len, i;
	int fd, ret, c;

	while ((c = getopt(argc, argv, "m:s:d:p:q:n:r:t:h")) != -1) {
		switch (c) {
		case 'm': o.dst_mac = optarg; break;
		case 's': o.src_ip = optarg; break;
		case 'd': o.dst_ip = optarg; break;
		case 'p': o.dst_port = (unsigned int)atoi(optarg); break;
		case 'q': o.src_port = (unsigned int)atoi(optarg); break;
		case 'n': o.payload = (unsigned int)atoi(optarg); break;
		case 'r': o.steer_port = (unsigned int)atoi(optarg); break;
		case 't': o.rx_ms = (unsigned int)atoi(optarg); break;
		default: usage();
		}
	}

	fd = open(DEV, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", DEV, strerror(errno));
		return 1;
	}

	if (ioctl(fd, GEMSLOT_GET_INFO, &info)) {
		fprintf(stderr, "GET_INFO: %s\n", strerror(errno));
		return 1;
	}

	printf("pool_phys  0x%08x\n", info.pool_phys);
	printf("pool_size  %u\n", info.pool_size);
	printf("slot_size  %u\n", info.slot_size);
	printf("slot_count %u\n", info.slot_count);
	printf("tx_offset  0x%08x\n", info.tx_offset);
	printf("rx_offset  0x%08x\n", info.rx_offset);

	pool = mmap(NULL, info.pool_size, PROT_READ | PROT_WRITE, MAP_SHARED,
		    fd, 0);
	if (pool == MAP_FAILED) {
		fprintf(stderr, "mmap: %s\n", strerror(errno));
		return 1;
	}

	if (read_if_mac("eth0", src_mac))
		fprintf(stderr, "cannot read eth0 MAC, sending from 00:..:00\n");

	len = build_frame(pool + info.tx_offset + GEMSLOT_FRAME_OFFSET, &o,
			  src_mac);

	tx.count = 1;
	tx.entries[0].slot = 0;
	tx.entries[0].len = (unsigned short)len;

	ret = ioctl(fd, GEMSLOT_TX_SUBMIT, &tx);
	if (ret < 0) {
		fprintf(stderr, "TX_SUBMIT: %s\n", strerror(errno));
		return 1;
	}
	printf("submitted %d frame(s) of %u bytes from transmit slot 0\n",
	       ret, len);

	memset(&wait, 0, sizeof(wait));
	wait.events = GEMSLOT_EVENT_TX;
	wait.timeout_ms = 1000;
	wait.tx_threshold = 1;
	if (ioctl(fd, GEMSLOT_WAIT, &wait))
		fprintf(stderr, "WAIT: %s\n", strerror(errno));

	memset(&tx, 0, sizeof(tx));
	ret = ioctl(fd, GEMSLOT_TX_COMPLETIONS, &tx);
	if (ret < 0) {
		fprintf(stderr, "TX_COMPLETIONS: %s\n", strerror(errno));
		return 1;
	}
	printf("completions %d\n", ret);
	for (i = 0; i < tx.count; i++)
		printf("  slot %u len %u\n", tx.entries[i].slot,
		       tx.entries[i].len);

	steer.udp_port = (unsigned short)o.steer_port;
	steer.enable = 1;
	if (ioctl(fd, GEMSLOT_RX_STEER, &steer)) {
		fprintf(stderr, "RX_STEER: %s\n", strerror(errno));
		return 1;
	}
	printf("steering UDP port %u for %u ms\n", o.steer_port, o.rx_ms);

	memset(&wait, 0, sizeof(wait));
	wait.events = GEMSLOT_EVENT_RX;
	wait.timeout_ms = o.rx_ms;
	wait.rx_threshold = 1;
	if (ioctl(fd, GEMSLOT_WAIT, &wait))
		fprintf(stderr, "WAIT: %s\n", strerror(errno));

	ret = ioctl(fd, GEMSLOT_RX_POLL, &rx);
	if (ret < 0) {
		fprintf(stderr, "RX_POLL: %s\n", strerror(errno));
		return 1;
	}
	printf("steered frames %d\n", ret);
	for (i = 0; i < rx.count; i++) {
		unsigned char *f = pool + info.rx_offset +
				   (unsigned int)rx.entries[i].slot *
				   info.slot_size + GEMSLOT_FRAME_OFFSET;

		printf("  slot %u len %u: ", rx.entries[i].slot,
		       rx.entries[i].len);
		dump(f, rx.entries[i].len);
	}

	if (rx.count && ioctl(fd, GEMSLOT_RX_RETURN, &rx) < 0)
		fprintf(stderr, "RX_RETURN: %s\n", strerror(errno));

	steer.enable = 0;
	ioctl(fd, GEMSLOT_RX_STEER, &steer);

	munmap(pool, info.pool_size);
	close(fd);

	return 0;
}
