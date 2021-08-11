/*
 * ice.h - ICE driver for Shenango's network stack
 */

#pragma once

#include "ice_rxtx.h"

#include "../defs.h"

struct ice_rxq {
	/* handle for runtime */
	struct hardware_q rxq;

	uint32_t consumer_idx;

	struct ice_rx_queue *dpdk_rxq;
} __aligned(CACHE_LINE_SIZE);

struct ice_txq {
	/* handle for runtime */
	struct direct_txq txq;

	struct ice_tx_queue *dpdk_txq;
} __aligned(CACHE_LINE_SIZE);

struct page_paddrs {
	void *base;
	size_t len;
	physaddr_t paddrs[];
} __aligned(CACHE_LINE_SIZE);

extern struct page_paddrs *rx_addrs;
extern struct page_paddrs *tx_addrs;
