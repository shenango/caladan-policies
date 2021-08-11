/*
 * ice_rxtx.c - RX/TX functions for ICE driver
 */

#if (defined(DIRECTPATH) && defined(ICE))

#include <base/compiler.h>
#include <base/log.h>
#include <base/thread.h>
#include <iokernel/queue.h>
#include <net/ethernet.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/udp.h>

#include "ice.h"
#include "ice_rxtx.h"

#define ICE_RX_FLEX_ERR0_BITS	\
	((1 << ICE_RX_FLEX_DESC_STATUS0_HBO_S) |	\
	 (1 << ICE_RX_FLEX_DESC_STATUS0_XSUM_IPE_S) |	\
	 (1 << ICE_RX_FLEX_DESC_STATUS0_XSUM_L4E_S) |	\
	 (1 << ICE_RX_FLEX_DESC_STATUS0_XSUM_EIPE_S) |	\
	 (1 << ICE_RX_FLEX_DESC_STATUS0_XSUM_EUDPE_S) |	\
	 (1 << ICE_RX_FLEX_DESC_STATUS0_RXE_S))

/* TEMP: hack to avoid initializing queues in the runtime */
static struct shm_region ingress_mbuf_region = {
	.base = (void *) 0x7fddaea00000,
	.len = INGRESS_MBUF_SHM_SIZE
};

/* RX L3/L4 checksum */
static inline uint64_t
ice_rxd_error_to_csum_type(uint16_t stat_err0)
{
	/* check if HW has decoded the packet and checksum, this will fail for
	   non-IP packets */
	if (unlikely(!(stat_err0 & (1 << ICE_RX_FLEX_DESC_STATUS0_L3L4P_S))))
		return CHECKSUM_TYPE_NEEDED;

	if (likely(!(stat_err0 & ICE_RX_FLEX_ERR0_BITS)))
		return CHECKSUM_TYPE_UNNECESSARY;

	if (unlikely(stat_err0 & (1 << ICE_RX_FLEX_DESC_STATUS0_XSUM_IPE_S))) {
		log_err_ratelimited("bad IP checksum, should drop packet");
		return CHECKSUM_TYPE_NEEDED; // TODO: or drop packet? checksum is bad
	}

	if (unlikely(stat_err0 & (1 << ICE_RX_FLEX_DESC_STATUS0_XSUM_L4E_S))) {
		log_err_ratelimited("bad L4 checksum, should drop packet");
		return CHECKSUM_TYPE_NEEDED; // TODO: or drop packet? checksum is bad
	}

	return CHECKSUM_TYPE_UNNECESSARY; /* L3 and L4 checksums are correct */
}

void empty_completion(struct mbuf *m)
{
	/* TEMP until RX queues are initialized properly */
}

int ice_recv_pkts(struct hardware_q *hw_q, struct mbuf **rx_pkts,
		unsigned int nb_pkts)
{
	struct ice_rxq *ice_rxq = container_of(hw_q, struct ice_rxq, rxq);
	struct ice_rx_queue *rxq = ice_rxq->dpdk_rxq;

	volatile union ice_rx_flex_desc *rx_ring = rxq->rx_ring;
	volatile union ice_rx_flex_desc *rxdp;
	union ice_rx_flex_desc rxd;
	struct ice_rx_entry *sw_ring = rxq->sw_ring;
	struct ice_rx_entry *rxe;
	struct mbuf *nmb; /* new allocated mbuf */
	struct mbuf *rxm; /* pointer to store old mbuf in SW ring */
	uint32_t rx_id = ice_rxq->consumer_idx;
	uint16_t nb_rx = 0;
	uint16_t nb_hold = 0;
	uint16_t rx_packet_len;
	volatile struct ice_32b_rx_flex_desc_comms *desc;
	uint16_t rx_stat_err0;
	uint64_t dma_addr;
	uint32_t page_number;
	physaddr_t phys_addr;
	shmptr_t shmptr;

	while (nb_rx < nb_pkts) {
		rxdp = &rx_ring[rx_id & (hw_q->nr_descriptors - 1)];

		rx_stat_err0 = le16_to_cpu(rxdp->wb.status_error0);
		/* check the DD bit first */
		if (!(rx_stat_err0 & (1 << ICE_RX_FLEX_DESC_STATUS0_DD_S)))
			break;

		/* allocate mbuf */
		nmb = (struct mbuf *) tcache_alloc(&perthread_get(directpath_buf_pt));
		if (unlikely(!nmb)) {
			log_err("ERROR: RX MBUF ALLOC FAILED\n");
			break;
		}
		rxd = *rxdp; /* copy descriptor in ring to temp variable*/

		nb_hold++;
		rxe = &sw_ring[rx_id & (hw_q->nr_descriptors - 1)]; /* get corresponding mbuf in SW ring */
		rx_id++;
		rxm = rxe->mbuf;
		rxe->mbuf = nmb;

		/* get DMA address by looking up page's physical address */
		page_number = PGN_2MB((uintptr_t) nmb - (uintptr_t) rx_addrs->base);
		phys_addr = rx_addrs->paddrs[page_number] + PGOFF_2MB(nmb);
		dma_addr = cpu_to_le64(phys_addr + RX_BUF_HEAD);

		/**
		 * fill the read format of descriptor with physical address in
		 * new allocated mbuf: nmb
		 */
		rxdp->read.hdr_addr = 0;
		rxdp->read.pkt_addr = dma_addr;

		/* calculate rx_packet_len of the received pkt */
		rx_packet_len = (le16_to_cpu(rxd.wb.pkt_len) &
				 ICE_RX_FLX_DESC_PKT_LEN_M) - rxq->crc_len;

		if ((void *) rxm >= ingress_mbuf_region.base &&
			(void *) rxm < ingress_mbuf_region.base + ingress_mbuf_region.len) {
			void *buf_addr;
			/* TEMP: hack because DPDK initialized these buffers,
			   determines the corresponding virtual address in our
			   mapping of the shared memory. fix once runtimes
			   initialize their own queues. */
			shmptr = ptr_to_shmptr(&ingress_mbuf_region, rxm, sizeof(*rxm));
			buf_addr = shmptr_to_ptr(&netcfg.rx_region, (shmptr_t) shmptr, MBUF_DEFAULT_LEN);
			shmptr = ptr_to_shmptr(&ingress_mbuf_region, *(void **) buf_addr, sizeof(*rxm));
			rxm = shmptr_to_ptr(&netcfg.rx_region, (shmptr_t) shmptr, MBUF_DEFAULT_LEN);
			rxm->release = empty_completion;
		} else
			rxm->release = directpath_rx_completion;

		/* fill old mbuf with received descriptor: rxd */
		prefetch0((unsigned char *) rxm + RX_BUF_HEAD);
		mbuf_init(rxm, (unsigned char *) rxm + RX_BUF_HEAD, rx_packet_len, 0);
		rxm->len = rx_packet_len;

		/* get RSS hash from packet if possible */
		desc = (volatile struct ice_32b_rx_flex_desc_comms *) &rxd;
		if (likely(le16_to_cpu(desc->status_error0) &
				(1 << ICE_RX_FLEX_DESC_STATUS0_RSS_VALID_S)))
			rxm->rss_hash = le32_to_cpu(desc->rss_hash);

		/* check if IP checksum was correct */
		rxm->csum_type = ice_rxd_error_to_csum_type(rx_stat_err0);
		rxm->csum = 0;

		/* copy old mbuf to rx_pkts */
		rx_pkts[nb_rx++] = rxm;
	}
	ice_rxq->consumer_idx = rx_id;

	/* update tail pointer for IOKernel */
	ACCESS_ONCE(*hw_q->shadow_tail) = ice_rxq->consumer_idx;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the receive tail register of queue.
	 * Update that register with the value of the last processed RX
	 * descriptor minus 1.
	 */
	nb_hold = (uint16_t)(nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		rx_id &= (hw_q->nr_descriptors - 1);
		rx_id = (uint16_t)(rx_id == 0 ?
				   (rxq->nb_rx_desc - 1) : (rx_id - 1));
		/* write TAIL register */
		ICE_PCI_REG_WRITE(rxq->qrx_tail, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;

	/* return received packet in the burst */
	return nb_rx;
}

static inline void
ice_txd_enable_checksum(uint64_t ol_flags,
			uint32_t *td_cmd,
			uint32_t *td_offset)
{
	/* set MACLEN */
	*td_offset |= (ETH_HDR_LEN >> 1)
		<< ICE_TX_DESC_LEN_MACLEN_S;

	/* enable L3 checksum offloads */
	if (ol_flags & OLFLAG_IP_CHKSUM) {
		*td_cmd |= ICE_TX_DESC_CMD_IIPT_IPV4_CSUM;
		*td_offset |= (sizeof(struct ip_hdr) >> 2) <<
			      ICE_TX_DESC_LEN_IPLEN_S;
	} else if (ol_flags & OLFLAG_IPV4) {
		*td_cmd |= ICE_TX_DESC_CMD_IIPT_IPV4;
		*td_offset |= (sizeof(struct ip_hdr) >> 2) <<
			      ICE_TX_DESC_LEN_IPLEN_S;
	}

	/* enable L4 checksum offloads */
	if (ol_flags & OLFLAG_TCP_CHKSUM) {
		*td_cmd |= ICE_TX_DESC_CMD_L4T_EOFT_TCP;
		*td_offset |= (sizeof(struct tcp_hdr) >> 2) <<
			      ICE_TX_DESC_LEN_L4_LEN_S;
	}
}

static inline int
ice_xmit_cleanup(struct ice_tx_queue *txq)
{
	struct ice_tx_entry *sw_ring = txq->sw_ring;
	volatile struct ice_tx_desc *txd = txq->tx_ring;
	uint16_t last_desc_cleaned = txq->last_desc_cleaned;
	uint16_t nb_tx_desc = txq->nb_tx_desc;
	uint16_t desc_to_clean_to;
	uint16_t nb_tx_to_clean;

	/* determine the last descriptor needing to be cleaned */
	desc_to_clean_to = (uint16_t)(last_desc_cleaned + txq->tx_rs_thresh);
	if (desc_to_clean_to >= nb_tx_desc)
		desc_to_clean_to = (uint16_t)(desc_to_clean_to - nb_tx_desc);

	/* check to make sure the last descriptor to clean is done */
	desc_to_clean_to = sw_ring[desc_to_clean_to].last_id;
	if (!(txd[desc_to_clean_to].cmd_type_offset_bsz &
	    cpu_to_le64(ICE_TX_DESC_DTYPE_DESC_DONE))) {
		log_warn("TX descriptor %4u is not done "
			"(port=%d queue=%d) value=0x%lx\n",
			desc_to_clean_to,
			txq->port_id, txq->queue_id,
			txd[desc_to_clean_to].cmd_type_offset_bsz);
		/* failed to clean any descriptors */
		return -1;
	}

	/* figure out how many descriptors will be cleaned */
	if (last_desc_cleaned > desc_to_clean_to)
		nb_tx_to_clean = (uint16_t)((nb_tx_desc - last_desc_cleaned) +
					    desc_to_clean_to);
	else
		nb_tx_to_clean = (uint16_t)(desc_to_clean_to -
					    last_desc_cleaned);

	/* The last descriptor to clean is done, so that means all the
	 * descriptors from the last descriptor that was cleaned
	 * up to the last descriptor with the RS bit set
	 * are done. Only reset the threshold descriptor.
	 */
	txd[desc_to_clean_to].cmd_type_offset_bsz = 0;

	/* update the txq to reflect the last descriptor that was cleaned */
	txq->last_desc_cleaned = desc_to_clean_to;
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free + nb_tx_to_clean);

	return 0;
}

int ice_xmit_pkt(struct mbuf *tx_pkt)
{
	struct ice_txq *ice_txq = container_of(myk()->directpath_txq,
					struct ice_txq, txq);
	struct ice_tx_queue *txq = ice_txq->dpdk_txq;
	volatile struct ice_tx_desc *tx_ring;
	volatile struct ice_tx_desc *txd;
	struct ice_tx_entry *sw_ring;
	struct ice_tx_entry *txe;
	uint16_t tx_id;
	uint32_t td_cmd = 0;
	uint32_t td_offset = 0;
	uint32_t td_tag = 0;
	uint16_t tx_last;
	unsigned char *data;
	uint32_t page_number;
	physaddr_t phys_addr;

	sw_ring = txq->sw_ring;
	tx_ring = txq->tx_ring;
	tx_id = txq->tx_tail;
	txe = &sw_ring[tx_id];

	/* check if the descriptor ring needs to be cleaned */
	if (txq->nb_tx_free < txq->tx_free_thresh)
		ice_xmit_cleanup(txq);
	if (txq->nb_tx_free == 0 && ice_xmit_cleanup(txq) != 0)
		return 1;

	/* circular ring */
	tx_last = (uint16_t) tx_id;
	if (tx_last >= txq->nb_tx_desc)
		tx_last = (uint16_t)(tx_last - txq->nb_tx_desc);

	/* enable checksum offloading */
	ice_txd_enable_checksum(tx_pkt->txflags, &td_cmd, &td_offset);

	/* free old mbuf if present */
	if (txe->mbuf)
		mbuf_free(txe->mbuf);
	txe->mbuf = tx_pkt;

	/* look up physical address for buffer's page */
	data = mbuf_data(tx_pkt);
	page_number = PGN_2MB((uintptr_t) data - (uintptr_t) tx_addrs->base);
	phys_addr = tx_addrs->paddrs[page_number] + PGOFF_2MB(data);

	/* setup TX Descriptor */
	txd = &tx_ring[tx_id];
	txd->buf_addr = cpu_to_le64(phys_addr);
	txd->cmd_type_offset_bsz =
		cpu_to_le64(ICE_TX_DESC_DTYPE_DATA |
			((uint64_t)td_cmd  << ICE_TXD_QW1_CMD_S) |
			((uint64_t)td_offset << ICE_TXD_QW1_OFFSET_S) |
			((uint64_t)mbuf_length(tx_pkt)  <<
				ICE_TXD_QW1_TX_BUF_SZ_S) |
			((uint64_t)td_tag  << ICE_TXD_QW1_L2TAG1_S));

	txe->last_id = tx_last;
	tx_id = txe->next_id;

	/* fill the last descriptor with End of Packet (EOP) bit */
	td_cmd |= ICE_TX_DESC_CMD_EOP;
	txq->nb_tx_used++;
	txq->nb_tx_free--;

	/* set RS bit on the last descriptor of one packet */
	if (txq->nb_tx_used >= txq->tx_rs_thresh) {
		td_cmd |= ICE_TX_DESC_CMD_RS;

		/* update txq RS bit counters */
		txq->nb_tx_used = 0;
	}
	txd->cmd_type_offset_bsz |=
		cpu_to_le64(((uint64_t)td_cmd) <<
				 ICE_TXD_QW1_CMD_S);

	/* update tail register */
	ICE_PCI_REG_WRITE(txq->qtx_tail, tx_id);
	txq->tx_tail = tx_id;

	/* prefetch next mbuf for freeing */
	txe = &sw_ring[tx_id];
	if (txe->mbuf)
		prefetch0((unsigned char *) txe->mbuf);

	return 0;
}

#endif
