
#ifdef DIRECTPATH

#include <util/mmio.h>
#include <util/udma_barrier.h>

#include "../../defs.h"
#include "mlx5.h"
#include "mlx5_ifc.h"

static int mlx5_qs_init_flows(unsigned int nr_rxq)
{
	int i;
	struct ibv_wq *ind_tbl[2048];

	for (i = 0; i < 2048; i++)
		ind_tbl[i] = rxqs[i % nr_rxq].rx_wq;

	return verbs_rss_init(context, pd, ind_tbl, 2048);
}

static int mlx5_qs_init_qs(unsigned int nr_rxq)
{
	int i;
	struct hardware_q *qs[nr_rxq];

	for (i = 0; i < nr_rxq; i++)
		qs[i] = &rxqs[i].rxq;

	return init_qs(qs, nr_rxq, mlx5_gather_rx);

}

static struct net_driver_ops mlx5_net_ops_queue_steering = {
	.rx_batch = qs_gather_rx,
	.tx_single = mlx5_transmit_one,
	.steer_flows = qs_steer,
	.register_flow = qs_register_flow,
	.deregister_flow = qs_deregister_flow,
	.get_flow_affinity = verbs_rss_flow_affinity,
	.rxq_has_work = qs_have_work,
};

int mlx5_init_queue_steering(struct hardware_q **rxq_out, struct direct_txq **txq_out,
	             unsigned int nr_rxq, unsigned int nr_txq)
{
	int ret;

	ret = mlx5_common_init(rxq_out, txq_out, nr_rxq, nr_txq, true);
	if (ret)
		return ret;

	ret = mlx5_qs_init_flows(nr_rxq);
	if (ret)
		return ret;

	ret = mlx5_qs_init_qs(nr_rxq);
	if (ret)
		return ret;

	net_ops = mlx5_net_ops_queue_steering;

	return 0;
}

#endif
