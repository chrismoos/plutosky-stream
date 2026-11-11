// SPDX-License-Identifier: GPL-2.0-only
/*
 * Host-side tests for the pure-logic parts of the receive path: the IPv4/UDP
 * classifier and the slot index rings.
 *
 * Build: cc -Wall -Wextra -I <linux>/drivers/net/ethernet/cadence \
 *        -o gemslot_lib_test gemslot_lib_test.c
 */

#include <stdio.h>
#include <string.h>

#include "macb_gemslot_lib.h"

#define GEMSLOT_TEST_RING	8
#define TEST_PORT		30002
#define OTHER_PORT		30003

static int failures;

static void check(int ok, const char *what)
{
	if (!ok) {
		printf("FAIL %s\n", what);
		failures++;
	} else {
		printf("ok   %s\n", what);
	}
}

static void put_be16(__u8 *p, unsigned int v)
{
	p[0] = (__u8)(v >> 8);
	p[1] = (__u8)v;
}

static unsigned int build(__u8 *f, unsigned int ethertype, unsigned int proto,
			  unsigned int dport, unsigned int frag, int vlan)
{
	unsigned int ip = GEMSLOT_ETH_HLEN;
	unsigned int udp;

	memset(f, 0, 128);
	if (vlan) {
		put_be16(f + GEMSLOT_ETHERTYPE_OFF, GEMSLOT_ETHERTYPE_VLAN);
		put_be16(f + GEMSLOT_ETH_HLEN, 0x0064);
		put_be16(f + GEMSLOT_ETH_HLEN + 2, ethertype);
		ip += GEMSLOT_VLAN_TAG_LEN;
	} else {
		put_be16(f + GEMSLOT_ETHERTYPE_OFF, ethertype);
	}

	f[ip] = 0x40 | GEMSLOT_IPV4_MIN_IHL;
	f[ip + 9] = (__u8)proto;
	put_be16(f + ip + 6, frag);

	udp = ip + GEMSLOT_IPV4_MIN_IHL * 4;
	put_be16(f + udp + 2, dport);

	return udp + GEMSLOT_UDP_HLEN;
}

static void test_classifier(void)
{
	__u8 map[GEMSLOT_PORTMAP_BYTES];
	__u8 f[128];
	unsigned int len;

	memset(map, 0, sizeof(map));
	gemslot_bit_assign(map, TEST_PORT, 1);

	check(gemslot_bit_test(map, TEST_PORT), "port set is present");
	check(!gemslot_bit_test(map, OTHER_PORT), "other port is absent");
	gemslot_bit_assign(map, TEST_PORT, 0);
	check(!gemslot_bit_test(map, TEST_PORT), "port clear removes it");
	gemslot_bit_assign(map, TEST_PORT, 1);

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, GEMSLOT_IPPROTO_UDP, TEST_PORT,
		    0, 0);
	check(gemslot_udp_dport(f, len) == TEST_PORT, "plain UDP port decodes");
	check(gemslot_steered(map, f, len), "matching port steers");

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, GEMSLOT_IPPROTO_UDP, OTHER_PORT,
		    0, 0);
	check(!gemslot_steered(map, f, len), "other port does not steer");

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, 6, TEST_PORT, 0, 0);
	check(gemslot_udp_dport(f, len) == -1, "TCP is not classified");

	len = build(f, 0x0806, GEMSLOT_IPPROTO_UDP, TEST_PORT, 0, 0);
	check(gemslot_udp_dport(f, len) == -1, "ARP is not classified");

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, GEMSLOT_IPPROTO_UDP, TEST_PORT,
		    0x0020, 0);
	check(gemslot_udp_dport(f, len) == -1, "later fragment is not classified");

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, GEMSLOT_IPPROTO_UDP, TEST_PORT,
		    0x4000, 0);
	check(gemslot_udp_dport(f, len) == TEST_PORT,
	      "don't-fragment frame still decodes");

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, GEMSLOT_IPPROTO_UDP, TEST_PORT,
		    0, 1);
	check(gemslot_udp_dport(f, len) == TEST_PORT, "VLAN tagged frame decodes");

	len = build(f, GEMSLOT_ETHERTYPE_IPV4, GEMSLOT_IPPROTO_UDP, TEST_PORT,
		    0, 0);
	check(gemslot_udp_dport(f, len - 1) == -1, "truncated frame is rejected");
	check(gemslot_udp_dport(f, 4) == -1, "runt frame is rejected");

	f[GEMSLOT_ETH_HLEN] = 0x60 | GEMSLOT_IPV4_MIN_IHL;
	check(gemslot_udp_dport(f, len) == -1, "IPv6 version is rejected");

	f[GEMSLOT_ETH_HLEN] = 0x40 | 3;
	check(gemslot_udp_dport(f, len) == -1, "short IHL is rejected");
}

static void test_ring(void)
{
	struct gemslot_ring r;
	__u16 slots[GEMSLOT_TEST_RING];
	unsigned int i;
	int idx;
	int ok;

	gemslot_ring_init(&r, GEMSLOT_TEST_RING);
	check(gemslot_ring_count(&r) == 0, "new ring is empty");
	check(gemslot_ring_pop(&r) == -1, "pop of an empty ring fails");

	ok = 1;
	for (i = 0; i < GEMSLOT_TEST_RING - 1; i++) {
		idx = gemslot_ring_push(&r);
		if (idx < 0) {
			ok = 0;
			break;
		}
		slots[idx] = (__u16)i;
	}
	check(ok, "ring holds capacity minus one entries");
	check(gemslot_ring_count(&r) == GEMSLOT_TEST_RING - 1,
	      "count matches what went in");
	check(gemslot_ring_space(&r) == 0, "full ring reports no space");
	check(gemslot_ring_push(&r) == -1, "push past capacity fails");

	ok = 1;
	for (i = 0; i < GEMSLOT_TEST_RING - 1; i++) {
		idx = gemslot_ring_pop(&r);
		if (idx < 0 || slots[idx] != (__u16)i) {
			ok = 0;
			break;
		}
	}
	check(ok, "entries come back in order");
	check(gemslot_ring_count(&r) == 0, "drained ring is empty");

	/* Wrap the indices many times over to catch a mask mistake. */
	ok = 1;
	for (i = 0; i < 1000; i++) {
		idx = gemslot_ring_push(&r);
		if (idx < 0) {
			ok = 0;
			break;
		}
		slots[idx] = (__u16)i;
		idx = gemslot_ring_pop(&r);
		if (idx < 0 || slots[idx] != (__u16)i) {
			ok = 0;
			break;
		}
	}
	check(ok, "push and pop survive wrapping");
}

int main(void)
{
	test_classifier();
	test_ring();

	if (failures) {
		printf("%d failure(s)\n", failures);
		return 1;
	}

	printf("all tests passed\n");

	return 0;
}
