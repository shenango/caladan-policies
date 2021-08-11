
#if (defined(DIRECTPATH) && defined(MLX4))

#include <base/log.h>
#include <base/mempool.h>

#include <util/mmio.h>
#include <util/udma_barrier.h>

#include "mlx4.h"

static struct ibv_context *context;
static struct ibv_pd *pd;
static struct ibv_mr *mr_tx;
static struct ibv_mr *mr_rx;

static struct mlx4_rxq rxqs[NCPU];
static struct mlx4_txq txqs[NCPU];

#define PORT_NUM 1 // TODO: make this dynamic

static void mlx4_init_tx_segment(struct mlx4_txq *v, unsigned int idx)
{
	struct mlx4dv_qp *qp = &v->tx_qp_dv;
	struct mlx4_wqe_ctrl_seg *ctrl;
	struct mlx4_wqe_data_seg *dpseg;
	void *segment;

	segment = qp->buf.buf + qp->sq.offset + (idx << qp->sq.wqe_shift);
	ctrl = segment;
	dpseg = segment + sizeof(*ctrl);

	ctrl->srcrb_flags = htobe32(MLX4_WQE_CTRL_CQ_UPDATE);
	/* For raw eth, the MLX4_WQE_CTRL_SOLICIT flag is used
	 * to indicate that no icrc should be calculated */
	ctrl->srcrb_flags |= htobe32(MLX4_WQE_CTRL_SOLICIT);

	ctrl->srcrb_flags |= htobe32(MLX4_WQE_CTRL_IP_HDR_CSUM |
							   MLX4_WQE_CTRL_TCP_UDP_CSUM);
	ctrl->imm = 0;
	dpseg->lkey = htobe32(mr_tx->lkey);
	ctrl->fence_size = (sizeof(*ctrl) + sizeof(*dpseg)) / 16;
}


/*
 * simple_alloc - simple memory allocator for internal MLX5 structures
 */
static void *simple_alloc(size_t size, void *priv_data)
{
	return iok_shm_alloc(size, PGSIZE_4KB, NULL);
}

static void simple_free(void *ptr, void *priv_data) {}

static struct mlx4dv_ctx_allocators dv_allocators = {
	.alloc = simple_alloc,
	.free = simple_free,
};

static int mlx4_create_rxq(int index, int rwq_wqn_alignment)
{
	int i, ret, rwq_discard = 0;
	unsigned char *buf;
	struct mlx4_rxq *v = &rxqs[index];
	struct mlx4dv_rwq *wq;
	struct mlx4_wqe_data_seg *seg;
	struct ibv_wq *to_free[rwq_wqn_alignment];
	struct ibv_wq_attr wq_attr;

	/* Create a CQ */
	struct ibv_cq_init_attr_ex cq_attr = {
		.cqe = RQ_NUM_DESC,
		.channel = NULL,
		.comp_vector = 0,
		.wc_flags = IBV_WC_EX_WITH_BYTE_LEN,
		.comp_mask = IBV_CQ_INIT_ATTR_MASK_FLAGS,
		.flags = IBV_CREATE_CQ_ATTR_SINGLE_THREADED,
	};
	v->rx_cq = ibv_create_cq_ex(context, &cq_attr);
	if (!v->rx_cq)
		return -errno;

	/* Create the work queue for RX */
	struct ibv_wq_init_attr wq_init_attr = {
		.wq_type = IBV_WQT_RQ,
		.max_wr = RQ_NUM_DESC,
		.max_sge = 1,
		.pd = pd,
		.cq = ibv_cq_ex_to_cq(v->rx_cq),
		.comp_mask = 0,
	};
	v->rx_wq = ibv_create_wq(context, &wq_init_attr);

	/* mlx4 wants the wqn to be aligned with the size of the RSS table */
	/* keep creating WQs until the alignment matches */
	if (rwq_wqn_alignment) {
		while (v->rx_wq && v->rx_wq->wq_num % rwq_wqn_alignment != 0) {
			to_free[rwq_discard++] = v->rx_wq;
			v->rx_wq = ibv_create_wq(context, &wq_init_attr);
		}

		for (i = 0; i < rwq_discard; i++)
			ibv_destroy_wq(to_free[i]);
	}

	if (!v->rx_wq)
		return -errno;

	if (wq_init_attr.max_wr != RQ_NUM_DESC)
		log_warn("Ring size is larger than anticipated");

	/* Set the WQ state to ready */
	memset(&wq_attr, 0, sizeof(wq_attr));
	wq_attr.attr_mask = IBV_WQ_ATTR_STATE;
	wq_attr.wq_state = IBV_WQS_RDY;
	ret = ibv_modify_wq(v->rx_wq, &wq_attr);
	if (ret)
		return -ret;

	/* expose direct verbs objects */
	struct mlx4dv_obj obj = {
		.cq = {
			.in = ibv_cq_ex_to_cq(v->rx_cq),
			.out = &v->rx_cq_dv,
		},
		.rwq = {
			.in = v->rx_wq,
			.out = &v->rx_wq_dv,
		},
	};
	ret = mlx4dv_init_obj(&obj, MLX4DV_OBJ_CQ | MLX4DV_OBJ_RWQ);
	if (ret)
		return -ret;

	BUG_ON(v->rx_cq_dv.cqe_size != 64);

	/* allocate list of posted buffers */
	v->buffers = aligned_alloc(CACHE_LINE_SIZE, v->rx_wq_dv.rq.wqe_cnt * sizeof(void *));
	if (!v->buffers)
		return -ENOMEM;

	v->rxq.consumer_idx = &v->consumer_idx;
	v->rxq.descriptor_table = v->rx_cq_dv.buf.buf;
	v->rxq.nr_descriptors = v->rx_cq_dv.cqe_cnt;
	v->rxq.descriptor_log_size = __builtin_ctz(v->rx_cq_dv.cqe_size);
	v->rxq.parity_byte_offset = sizeof(struct mlx4_cqe) + offsetof(struct mlx4_cqe, owner_sr_opcode);
	v->rxq.parity_bit_mask = MLX4_CQE_OWNER_MASK;

	/* set byte_count and lkey for all descriptors once */
	wq = &v->rx_wq_dv;
	for (i = 0; i < wq->rq.wqe_cnt; i++) {
		seg = wq->buf.buf + wq->rq.offset + (i << wq->rq.wqe_shift);
		seg->byte_count =  htobe32(MBUF_DEFAULT_LEN - RX_BUF_HEAD);
		seg->lkey = htobe32(mr_rx->lkey);

		/* fill queue with buffers */
		buf = mempool_alloc(&directpath_buf_mp);
		if (!buf)
			return -ENOMEM;

		seg->addr = htobe64((unsigned long)buf + RX_BUF_HEAD);
		v->buffers[i] = buf;
		v->wq_head++;
	}

	udma_to_device_barrier();
	*wq->rdb = htobe32(v->wq_head & 0xffff);

	return 0;
}

static int mlx4_init_txq(int index, struct mlx4_txq *v)
{
	int i, ret;

	/* Create a CQ */
	struct ibv_cq_init_attr_ex cq_attr = {
		.cqe = SQ_NUM_DESC,
		.channel = NULL,
		.comp_vector = 0,
		.wc_flags = 0,
		.comp_mask = IBV_CQ_INIT_ATTR_MASK_FLAGS,
		.flags = IBV_CREATE_CQ_ATTR_SINGLE_THREADED,
	};
	v->tx_cq = ibv_create_cq_ex(context, &cq_attr);
	if (!v->tx_cq)
		return -errno;

	/* Create a 1-sided queue pair for sending packets */
	struct ibv_qp_init_attr_ex qp_init_attr = {
		.send_cq = ibv_cq_ex_to_cq(v->tx_cq),
		.recv_cq = ibv_cq_ex_to_cq(v->tx_cq),
		.cap = {
			.max_send_wr = SQ_NUM_DESC,
			.max_recv_wr = 0,
			.max_send_sge = 1,
			.max_inline_data = 0, // TODO: should inline some data?
		},
		.qp_type = IBV_QPT_RAW_PACKET,
		.sq_sig_all = 1,
		.pd = pd,
		.comp_mask = IBV_QP_INIT_ATTR_PD
	};
	struct mlx4dv_qp_init_attr dv_qp_attr = {
		.comp_mask = 0,
	};
	v->tx_qp = mlx4dv_create_qp(context, &qp_init_attr, &dv_qp_attr);
	if (!v->tx_qp)
		return -errno;

	/* Turn on TX QP in 3 steps */
	struct ibv_qp_attr qp_attr;
	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state = IBV_QPS_INIT;
	qp_attr.port_num = 1;
	ret = ibv_modify_qp(v->tx_qp, &qp_attr, IBV_QP_STATE | IBV_QP_PORT);
	if (ret)
		return -ret;

	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state = IBV_QPS_RTR;
	ret = ibv_modify_qp(v->tx_qp, &qp_attr, IBV_QP_STATE);
	if (ret)
		return -ret;

	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state = IBV_QPS_RTS;
	ret = ibv_modify_qp(v->tx_qp, &qp_attr, IBV_QP_STATE);
	if (ret)
		return -ret;

	struct mlx4dv_obj obj = {
		.cq = {
			.in = ibv_cq_ex_to_cq(v->tx_cq),
			.out = &v->tx_cq_dv,
		},
		.qp = {
			.in = v->tx_qp,
			.out = &v->tx_qp_dv,
		},
	};
	ret = mlx4dv_init_obj(&obj, MLX4DV_OBJ_CQ | MLX4DV_OBJ_QP);
	if (ret)
		return -ret;

	BUG_ON(v->tx_cq_dv.cqe_size != 64);

	/* allocate list of posted buffers */
	v->buffers = aligned_alloc(CACHE_LINE_SIZE, v->tx_qp_dv.sq.wqe_cnt * sizeof(*v->buffers));
	if (!v->buffers)
		return -ENOMEM;

	for (i = 0; i < v->tx_qp_dv.sq.wqe_cnt; i++)
		mlx4_init_tx_segment(v, i);

	return 0;
}

static int mlx4_qs_init_flows(unsigned int nr_rxq)
{
	int i, ret;
	struct ibv_wq *ind_tbl[nr_rxq];

	for (i = 0; i < nr_rxq; i++)
		ind_tbl[i] = rxqs[i].rx_wq;

	ret = verbs_rss_init(context, pd, ind_tbl, nr_rxq);
	if (ret)
		return ret;

	return 0;
}

static int mlx4_qs_init_qs(unsigned int nr_rxq)
{
	int i;
	struct hardware_q *qs[nr_rxq];

	for (i = 0; i < nr_rxq; i++)
		qs[i] = &rxqs[i].rxq;

	return init_qs(qs, nr_rxq, mlx4_gather_rx);
}

static struct net_driver_ops mlx4_net_ops_queue_steering = {
	.rx_batch = qs_gather_rx,
	.tx_single = mlx4_transmit_one,
	.steer_flows = qs_steer,
	.register_flow = qs_register_flow,
	.deregister_flow = qs_deregister_flow,
	.get_flow_affinity = verbs_rss_flow_affinity,
	.rxq_has_work = qs_have_work,
};

/*
 * mlx4_init - intialize all TX/RX queues
 */
int mlx4_init(struct hardware_q **rxq_out, struct direct_txq **txq_out,
	             unsigned int nr_rxq, unsigned int nr_txq)
{
	int i, ret;

	struct ibv_device **dev_list;
	struct ibv_device *ib_dev;

	if (!is_power_of_two(nr_rxq) || nr_rxq > NCPU)
		return -EINVAL;

	dev_list = ibv_get_device_list(NULL);
	if (!dev_list) {
		perror("Failed to get IB devices list");
		return -1;
	}

	i = 0;
	while ((ib_dev = dev_list[i])) {
		if (strncmp(ibv_get_device_name(ib_dev), "mlx4", 4) == 0)
			break;
		i++;
	}

	if (!ib_dev) {
		log_err("mlx4_init: IB device not found");
		return -1;
	}

	context = ibv_open_device(ib_dev);

	if (!context) {
		log_err("mlx4_init: Couldn't get context for %s: errno = %d",
			ibv_get_device_name(ib_dev), errno);
		return -1;
	}

	ibv_free_device_list(dev_list);

	ret = mlx4dv_set_context_attr(context,
		  MLX4DV_SET_CTX_ATTR_BUF_ALLOCATORS, &dv_allocators);
	if (ret) {
		log_err("mlx4_init: error setting memory allocator");
		return -1;
	}

	pd = ibv_alloc_pd(context);
	if (!pd) {
		log_err("mlx4_init: Couldn't allocate PD");
		return -1;
	}

	/* Register memory for TX buffers */
	mr_tx = ibv_reg_mr(pd, net_tx_buf_mp.buf, net_tx_buf_mp.len, IBV_ACCESS_LOCAL_WRITE);
	if (!mr_tx) {
		log_err("mlx4_init: Couldn't register mr");
		return -1;
	}

	mr_rx = ibv_reg_mr(pd, directpath_buf_mp.buf, directpath_buf_mp.len, IBV_ACCESS_LOCAL_WRITE);
	if (!mr_rx) {
		log_err("mlx4_init: Couldn't register mr");
		return -1;
	}

	for (i = 0; i < nr_rxq; i++) {
		ret = mlx4_create_rxq(i, i ? 0 : nr_rxq);
		if (ret) {
			log_err("mlx4_init: failed to create rxq");
			return ret;
		}

		rxq_out[i] = &rxqs[i].rxq;
	}

	ret = mlx4_qs_init_flows(nr_rxq);
	if (ret)
		return ret;

	ret = mlx4_qs_init_qs(nr_rxq);
	if (ret)
		return ret;

	for (i = 0; i < nr_txq; i++) {
		ret = mlx4_init_txq(i, &txqs[i]);
		if (ret)
			return ret;

		txq_out[i] = &txqs[i].txq;
	}

	net_ops = mlx4_net_ops_queue_steering;

	return 0;
}



#endif
