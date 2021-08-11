/*
 * ice_rxtx.h - RX/TX functions for ICE driver
 */

#pragma once

#include <net/mbuf.h>

#include "../defs.h"
#include "ice_lan_tx_rx.h"

#define ICE_RX_MAX_BURST 32
#define ice_rx_flex_desc ice_32b_rx_flex_desc

struct ice_rx_queue;
struct ice_tx_queue;

typedef void (*ice_rx_release_mbufs_t)(struct ice_rx_queue *rxq);
typedef void (*ice_tx_release_mbufs_t)(struct ice_tx_queue *txq);

/* TEMP: fake out DPDK structs */
struct rte_mempool;
struct rte_mbuf {
	char pad[128];
};
typedef uint64_t rte_iova_t;

struct ice_rx_entry {
	struct mbuf *mbuf;
};

struct ice_rx_queue {
	struct rte_mempool *mp; /* mbuf pool to populate RX ring */
	volatile union ice_rx_flex_desc *rx_ring;/* RX ring virtual address */
	rte_iova_t rx_ring_dma; /* RX ring DMA address */
	struct ice_rx_entry *sw_ring; /* address of RX soft ring */
	uint16_t nb_rx_desc; /* number of RX descriptors */
	uint16_t rx_free_thresh; /* max free RX desc to hold */
	uint16_t rx_tail; /* current value of tail */
	uint16_t nb_rx_hold; /* number of held free RX desc */
	struct rte_mbuf *pkt_first_seg; /**< first segment of current packet */
	struct rte_mbuf *pkt_last_seg; /**< last segment of current packet */
	uint16_t rx_nb_avail; /**< number of staged packets ready */
	uint16_t rx_next_avail; /**< index of next staged packets */
	uint16_t rx_free_trigger; /**< triggers rx buffer allocation */
	struct rte_mbuf fake_mbuf; /**< dummy mbuf */
	struct rte_mbuf *rx_stage[ICE_RX_MAX_BURST * 2];

	uint16_t rxrearm_nb;	/**< number of remaining to be re-armed */
	uint16_t rxrearm_start;	/**< the idx we start the re-arming from */
	uint64_t mbuf_initializer; /**< value to init mbufs */

	uint8_t port_id; /* device port ID */
	uint8_t crc_len; /* 0 if CRC stripped, 4 otherwise */
	uint16_t queue_id; /* RX queue index */
	uint16_t reg_idx; /* RX queue register index */
	uint8_t drop_en; /* if not 0, set register bit */
	volatile uint8_t *qrx_tail; /* register address of tail */
	struct ice_vsi *vsi; /* the VSI this queue belongs to */
	uint16_t rx_buf_len; /* The packet buffer size */
	uint16_t rx_hdr_len; /* The header buffer size */
	uint16_t max_pkt_len; /* Maximum packet length */
	bool q_set; /* indicate if rx queue has been configured */
	bool rx_deferred_start; /* don't start this queue in dev start */
	uint8_t proto_xtr; /* Protocol extraction from flexible descriptor */
	ice_rx_release_mbufs_t rx_rel_mbufs;
};

struct ice_tx_entry {
	struct mbuf *mbuf;
	uint16_t next_id;
	uint16_t last_id;
};

struct ice_tx_queue {
	uint16_t nb_tx_desc; /* number of TX descriptors */
	rte_iova_t tx_ring_dma; /* TX ring DMA address */
	volatile struct ice_tx_desc *tx_ring; /* TX ring virtual address */
	struct ice_tx_entry *sw_ring; /* virtual address of SW ring */
	uint16_t tx_tail; /* current value of tail register */
	volatile uint8_t *qtx_tail; /* register address of tail */
	uint16_t nb_tx_used; /* number of TX desc used since RS bit set */
	/* index to last TX descriptor to have been cleaned */
	uint16_t last_desc_cleaned;
	/* Total number of TX descriptors ready to be allocated. */
	uint16_t nb_tx_free;
	/* Start freeing TX buffers if there are less free descriptors than
	 * this value.
	 */
	uint16_t tx_free_thresh;
	/* Number of TX descriptors to use before RS bit is set. */
	uint16_t tx_rs_thresh;
	uint8_t pthresh; /**< Prefetch threshold register. */
	uint8_t hthresh; /**< Host threshold register. */
	uint8_t wthresh; /**< Write-back threshold reg. */
	uint8_t port_id; /* Device port identifier. */
	uint16_t queue_id; /* TX queue index. */
	uint32_t q_teid; /* TX schedule node id. */
	uint16_t reg_idx;
	uint64_t offloads;
	struct ice_vsi *vsi; /* the VSI this queue belongs to */
	uint16_t tx_next_dd;
	uint16_t tx_next_rs;
	bool tx_deferred_start; /* don't start this queue in dev start */
	bool q_set; /* indicate if tx queue has been configured */
	ice_tx_release_mbufs_t tx_rel_mbufs;
};

int ice_recv_pkts(struct hardware_q *hw_q, struct mbuf **rx_pkts,
		       unsigned int nb_pkts);
int ice_xmit_pkt(struct mbuf *tx_pkt);
