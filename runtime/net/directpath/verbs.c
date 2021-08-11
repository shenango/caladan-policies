
#if (defined(DIRECTPATH) && (defined(MLX4) || defined(MLX5)))

#include <infiniband/verbs.h>
#include "../../defs.h"

#define PORT_NUM 1 // TODO: make this dynamic

static unsigned char rss_key[40] = {
	0x82, 0x19, 0xFA, 0x80, 0xA4, 0x31, 0x06, 0x59, 0x3E, 0x3F, 0x9A,
	0xAC, 0x3D, 0xAE, 0xD6, 0xD9, 0xF5, 0xFC, 0x0C, 0x63, 0x94, 0xBF,
	0x8F, 0xDE, 0xD2, 0xC5, 0xE2, 0x04, 0xB1, 0xCF, 0xB1, 0xB1, 0xA1,
	0x0D, 0x6D, 0x86, 0xBA, 0x61, 0x78, 0xEB};

static uint32_t tbl_sz;

uint32_t verbs_rss_flow_affinity(uint8_t ipproto, uint16_t local_port, struct netaddr remote)
{
	uint32_t i, j, map, ret = 0, input_tuple[] = {
		remote.ip, netcfg.addr, local_port | remote.port << 16
	};

	for (j = 0; j < ARRAY_SIZE(input_tuple); j++) {
		for (map = input_tuple[j]; map; map &= (map - 1)) {
			i = (uint32_t)__builtin_ctz(map);
			ret ^= hton32(((const uint32_t *)rss_key)[j]) << (31 - i) |
			       (uint32_t)((uint64_t)(hton32(((const uint32_t *)rss_key)[j + 1])) >>
			       (i + 1));
		}
	}

	return (ret & (tbl_sz - 1)) % (uint32_t)maxks;
}

int verbs_rss_init(struct ibv_context *context, struct ibv_pd *pd,
	struct ibv_wq **ind_tbl, unsigned int sz)
{
	int ret;
	struct ibv_flow *eth_flow;
	struct ibv_qp *tcp_qp, *other_qp;
	struct ibv_rwq_ind_table *rwq_ind_table;

	tbl_sz = sz;

	/* Create Receive Work Queue Indirection Table */
	struct ibv_rwq_ind_table_init_attr rwq_attr = {
		.log_ind_tbl_size = __builtin_ctz(sz),
		.ind_tbl = ind_tbl,
		.comp_mask = 0,
	};
	rwq_ind_table = ibv_create_rwq_ind_table(context, &rwq_attr);

	if (!rwq_ind_table)
		return -errno;

	/* Create the main RX QP using the indirection table */
	struct ibv_rx_hash_conf rss_cnf = {
		.rx_hash_function = IBV_RX_HASH_FUNC_TOEPLITZ,
		.rx_hash_key_len = ARRAY_SIZE(rss_key),
		.rx_hash_key = rss_key,
		.rx_hash_fields_mask = IBV_RX_HASH_SRC_IPV4 | IBV_RX_HASH_DST_IPV4 | IBV_RX_HASH_SRC_PORT_TCP | IBV_RX_HASH_DST_PORT_TCP,
	};

	struct ibv_qp_init_attr_ex qp_ex_attr = {
		.qp_type = IBV_QPT_RAW_PACKET,
		.comp_mask =  IBV_QP_INIT_ATTR_IND_TABLE | IBV_QP_INIT_ATTR_RX_HASH | IBV_QP_INIT_ATTR_PD,
		.pd = pd,
		.rwq_ind_tbl = rwq_ind_table,
		.rx_hash_conf = rss_cnf,
	};

	tcp_qp = ibv_create_qp_ex(context, &qp_ex_attr);
	if (!tcp_qp)
		return -errno;

	rss_cnf.rx_hash_fields_mask = IBV_RX_HASH_SRC_IPV4 | IBV_RX_HASH_DST_IPV4 | IBV_RX_HASH_SRC_PORT_UDP | IBV_RX_HASH_DST_PORT_UDP,
	qp_ex_attr.rx_hash_conf = rss_cnf;
	other_qp = ibv_create_qp_ex(context, &qp_ex_attr);
	if (!other_qp)
		return -errno;

	/* Turn on QP in 2 steps */
	// this only matters for mlx4. mlx5 returns ENOSYS
	struct ibv_qp_attr qp_attr = {0};
	qp_attr.qp_state = IBV_QPS_INIT;
	qp_attr.port_num = 1;
	ret = ibv_modify_qp(tcp_qp, &qp_attr, IBV_QP_STATE | IBV_QP_PORT);
	if (ret && ret != ENOSYS)
		return -ret;

	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state = IBV_QPS_INIT;
	qp_attr.port_num = 1;
	ret = ibv_modify_qp(other_qp, &qp_attr, IBV_QP_STATE | IBV_QP_PORT);
	if (ret && ret != ENOSYS)
		return -ret;

	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state = IBV_QPS_RTR;
	ret = ibv_modify_qp(tcp_qp, &qp_attr, IBV_QP_STATE);
	if (ret && ret != ENOSYS)
		return -ret;

	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state = IBV_QPS_RTR;
	ret = ibv_modify_qp(other_qp, &qp_attr, IBV_QP_STATE);
	if (ret && ret != ENOSYS)
		return -ret;

	/* Route TCP packets for our MAC address to the QP with TCP RSS configuration */
	struct raw_eth_flow_attr {
		struct ibv_flow_attr attr;
		struct ibv_flow_spec_eth spec_eth;
		struct ibv_flow_spec_tcp_udp spec_tcp;
	} __attribute__((packed)) flow_attr = {
		.attr = {
			.comp_mask = 0,
			.type = IBV_FLOW_ATTR_NORMAL,
			.size = sizeof(flow_attr),
			.priority = 0,
			.num_of_specs = 2,
			.port = PORT_NUM,
			.flags = 0,
		},
		.spec_eth = {
			.type = IBV_FLOW_SPEC_ETH,
			.size = sizeof(struct ibv_flow_spec_eth),
			.val = {
				.src_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
				.ether_type = 0,
				.vlan_tag = 0,
			},
			.mask = {
				.dst_mac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
				.src_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
				.ether_type = 0,
				.vlan_tag = 0,
			}
		},
		.spec_tcp = {
			.type = IBV_FLOW_SPEC_TCP,
			.size = sizeof(struct ibv_flow_spec_tcp_udp),
			.val = {0},
			.mask = {0},
		},
	};
	memcpy(&flow_attr.spec_eth.val.dst_mac, netcfg.mac.addr, 6);
	eth_flow = ibv_create_flow(tcp_qp, &flow_attr.attr);
	if (!eth_flow)
		return -errno;

	/* Route other unicast packets to the QP with the UDP RSS configuration */
	flow_attr.attr.num_of_specs = 1;
	eth_flow = ibv_create_flow(other_qp, &flow_attr.attr);
	if (!eth_flow)
		return -errno;

	/* Route broadcst packets to our set of RX work queues */
	memset(&flow_attr.spec_eth.val.dst_mac, 0xff, 6);
	eth_flow = ibv_create_flow(other_qp, &flow_attr.attr);
	if (!eth_flow)
		return -errno;

	/* Route multicast traffic to our RX queues */
	struct ibv_flow_attr mc_attr = {
		.comp_mask = 0,
		.type = IBV_FLOW_ATTR_MC_DEFAULT,
		.size = sizeof(mc_attr),
		.priority = 0,
		.num_of_specs = 0,
		.port = PORT_NUM,
		.flags = 0,
	};
	eth_flow = ibv_create_flow(other_qp, &mc_attr);
	if (!eth_flow)
		return -errno;

	return 0;
}

#endif