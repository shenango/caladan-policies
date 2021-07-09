#ifdef DIRECTPATH

#include <base/log.h>

#include <util/mmio.h>
#include <util/udma_barrier.h>


#include "../../defs.h"
#include "mlx4.h"

/*
 * mlx4_refill_rxqueue - replenish RX queue with nrdesc bufs
 * @vq: queue to refill
 * @nrdesc: number of buffers to fill
 *
 * WARNING: nrdesc must not exceed the number of free slots in the RXq
 * returns 0 on success, errno on error
 */

static inline int mlx4_refill_rxqueue(struct mlx4_rxq *vq, int nrdesc)
{
	unsigned int i;
	uint32_t index;
	unsigned char *buf;
	struct mlx4_wqe_data_seg *seg;

	struct mlx4dv_rwq *wq = &vq->rx_wq_dv;

	assert(nrdesc + vq->wq_head >= vq->consumer_idx + wq->rq.wqe_cnt);

	for (i = 0; i < nrdesc; i++) {
		buf = tcache_alloc(&perthread_get(directpath_buf_pt));
		if (unlikely(!buf))
			return -ENOMEM;

		index = vq->wq_head++ & (wq->rq.wqe_cnt - 1);
		seg = wq->buf.buf + wq->rq.offset + (index << wq->rq.wqe_shift);
		seg->addr = htobe64((unsigned long)buf + RX_BUF_HEAD);
		vq->buffers[index] = buf;
	}

	udma_to_device_barrier();
	*wq->rdb = htobe32(vq->wq_head & 0xffff);

	return 0;
}


/*
 * mlx4_gather_completions - collect up to budget received packets and completions
 */
static int mlx4_gather_completions(struct mbuf **mbufs, struct mlx4_txq *v, unsigned int budget)
{
	struct mlx4dv_cq *cq = &v->tx_cq_dv;
	struct mlx4_cqe *cqe, *cqes = cq->buf.buf;

	unsigned int compl_cnt;
	uint16_t wqe_idx;

	for (compl_cnt = 0; compl_cnt < budget; compl_cnt++, v->cq_head++) {
		cqe = &cqes[2 * (v->cq_head & (cq->cqe_cnt - 1))] + 1;

		if (!!(ACCESS_ONCE(cqe->owner_sr_opcode) & MLX4_CQE_OWNER_MASK) ^ !!(v->cq_head & cq->cqe_cnt))
			break;

		BUG_ON(mlx4dv_get_cqe_opcode(cqe) == MLX4_CQE_OPCODE_ERROR);

		wqe_idx = be16toh(cqe->wqe_index) & (v->tx_qp_dv.sq.wqe_cnt - 1);
		mbufs[compl_cnt] = v->buffers[wqe_idx];
	}

	*cq->set_ci_db = htobe32(v->cq_head  & 0xffffff);

	return compl_cnt;
}

/*
 * mlx4_transmit_one - send one mbuf
 * @t: queue to use
 * @m: mbuf to send
 *
 * returns 0 on success, errno on error
 */
int mlx4_transmit_one(struct mbuf *m)
{
	struct mlx4_txq *v = container_of(myk()->directpath_txq, struct mlx4_txq, txq);
	int i, compl = 0;
	struct mbuf *mbs[SQ_CLEAN_MAX];
	struct mlx4dv_qp *qp = &v->tx_qp_dv;

	struct mlx4_wqe_ctrl_seg *ctrl;
	struct mlx4_wqe_data_seg *dpseg;

	uint32_t idx = v->sq_head & (v->tx_qp_dv.sq.wqe_cnt - 1);

	if (nr_inflight_tx(v) >= SQ_CLEAN_THRESH) {
		compl = mlx4_gather_completions(mbs, v, SQ_CLEAN_MAX);
		for (i = 0; i < compl; i++)
			mbuf_free(mbs[i]);
		if (unlikely(nr_inflight_tx(v) >= qp->sq.wqe_cnt)) {
			log_warn_ratelimited("txq full");
			return 1;
		}
	}

	ctrl = qp->buf.buf + qp->sq.offset + (idx << qp->sq.wqe_shift);
	dpseg = (void *)ctrl + sizeof(*ctrl);

	/* mac address goes into descriptor for loopback */
	ctrl->srcrb_flags16[0] = *(__be16 *)(uintptr_t)mbuf_data(m);
	ctrl->imm = *(__be32 *)((uintptr_t)(mbuf_data(m)) + 2);

	dpseg->addr = htobe64((uint64_t)mbuf_data(m));

	// assuming we dont straddle cache lines
	BUILD_ASSERT(sizeof(*ctrl) + sizeof(*dpseg) <= CACHE_LINE_SIZE);

	dpseg->byte_count = htobe32(mbuf_length(m));

	udma_to_device_barrier();

	ctrl->owner_opcode = htobe32(MLX4_OPCODE_SEND) |
			(v->sq_head & qp->sq.wqe_cnt ? htobe32(1 << 31) : 0);

	udma_to_device_barrier();

	mmio_write32_be(qp->sdb, qp->doorbell_qpn);
	v->buffers[v->sq_head++ & (v->tx_qp_dv.sq.wqe_cnt - 1)] = m;
	return 0;
}

static inline bool mlx4_csum_ok(struct mlx4_cqe *cqe)
{
	return (cqe->status & htobe32(MLX4_CQE_STATUS_IPV4_CSUM_OK)) ==
				 htobe32(MLX4_CQE_STATUS_IPV4_CSUM_OK);
}

static inline void mbuf_fill_cqe(struct mbuf *m, struct mlx4_cqe *cqe)
{
	uint32_t len;

	len = be32toh(cqe->byte_cnt);

	mbuf_init(m, (unsigned char *)m + RX_BUF_HEAD, len, 0);
	m->len = len;

	m->csum_type = mlx4_csum_ok(cqe);
	m->csum = 0;
	m->rss_hash = cqe->immed_rss_invalid;

	m->release = directpath_rx_completion;
}

int mlx4_gather_rx(struct hardware_q *rxq, struct mbuf **ms, unsigned int budget)
{
	uint16_t wqe_idx;
	int rx_cnt;

	struct mlx4_rxq *v = container_of(rxq, struct mlx4_rxq, rxq);
	struct mlx4dv_rwq *wq = &v->rx_wq_dv;
	struct mlx4dv_cq *cq = &v->rx_cq_dv;

	struct mlx4_cqe *cqe, *cqes = cq->buf.buf;
	struct mbuf *m;

	for (rx_cnt = 0; rx_cnt < budget; rx_cnt++, v->consumer_idx++) {
		cqe = &cqes[2 * (v->consumer_idx & (cq->cqe_cnt - 1))] + 1;

		if (!!(ACCESS_ONCE(cqe->owner_sr_opcode) & MLX4_CQE_OWNER_MASK) ^ !!(v->consumer_idx & cq->cqe_cnt))
			break;

		BUG_ON(mlx4dv_get_cqe_opcode(cqe) == MLX4_CQE_OPCODE_ERROR);

		wqe_idx = be16toh(cqe->wqe_index) & (wq->rq.wqe_cnt - 1);
		m = v->buffers[wqe_idx];
		mbuf_fill_cqe(m, cqe);
		ms[rx_cnt] = m;
	}

	if (unlikely(!rx_cnt))
		return rx_cnt;

	ACCESS_ONCE(*rxq->shadow_tail) = v->consumer_idx;

	*cq->set_ci_db = htobe32(v->consumer_idx  & 0xffffff);
	BUG_ON(mlx4_refill_rxqueue(v, rx_cnt));

	return rx_cnt;
}

#endif