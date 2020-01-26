/*
 * mlx4.h - MLX4 driver for Shenango's network statck
 */


#pragma once

#include <infiniband/mlx4dv.h>
#include <infiniband/verbs.h>
#include <base/byteorder.h>

#include "../defs.h"

struct mlx4_rxq {
	/* handle for runtime */
	struct hardware_q rxq;

	uint32_t consumer_idx;

	struct mlx4dv_cq rx_cq_dv;
	struct mlx4dv_rwq rx_wq_dv;
	uint32_t wq_head;

	void **buffers; // array of posted buffers

	struct ibv_cq_ex *rx_cq;
	struct ibv_wq *rx_wq;

} __aligned(CACHE_LINE_SIZE);

struct mlx4_txq {
	/* handle for runtime */
	struct direct_txq txq;

	/* direct verbs qp */
	struct mbuf **buffers; // pending DMA
	struct mlx4dv_qp tx_qp_dv;
	uint32_t sq_head;

	/* direct verbs cq */
	struct mlx4dv_cq tx_cq_dv;
	uint32_t cq_head;

	struct ibv_cq_ex *tx_cq;
	struct ibv_qp *tx_qp;


} __aligned(CACHE_LINE_SIZE);

static inline unsigned int nr_inflight_tx(struct mlx4_txq *v)
{
	return v->sq_head - v->cq_head;
}

extern int mlx4_gather_rx(struct hardware_q *rxq, struct mbuf **ms, unsigned int budget);
extern int mlx4_transmit_one(struct mbuf *m);
