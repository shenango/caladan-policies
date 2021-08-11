/*
 * ice_init.c - initialization functions for ICE driver
 */

#if (defined(DIRECTPATH) && defined(ICE))

/* TEMP for mapping memory */
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>

#include <base/log.h>

#include "ice.h"

static struct ice_rxq rxqs[NCPU];
static struct ice_txq txqs[NCPU];

/* TEMP state set up by DPDK */
struct rte_eth_dev_data {
	char name[64];
	void **rx_queues;
	void **tx_queues;
};
struct rte_eth_dev_data *dpdk_data = (struct rte_eth_dev_data *) 0x1100bb0440;

struct page_paddrs *rx_addrs;
struct page_paddrs *tx_addrs;

static int ice_create_rxq(struct ice_rxq *rxq, int index)
{
	/* TEMP RX queue state set up by DPDK */
	struct ice_rx_queue *dpdk_rxq = dpdk_data->rx_queues[index];

	// init consumer_idx from DPDK queue
	rxq->consumer_idx = dpdk_rxq->rx_tail;

	rxq->rxq.descriptor_table = (void *) dpdk_rxq->rx_ring;
	rxq->rxq.consumer_idx = &rxq->consumer_idx;
	/* sizeof(union ice_rx_flex_desc) = 2^5 */
	rxq->rxq.descriptor_log_size = 5;
	rxq->rxq.nr_descriptors = dpdk_rxq->nb_rx_desc;
	rxq->rxq.parity_byte_offset = offsetof(union ice_rx_flex_desc, wb.status_error0);
	rxq->rxq.parity_bit_mask = (1 << ICE_RX_FLEX_DESC_STATUS0_DD_S);

	rxq->dpdk_rxq = dpdk_rxq;

	return 0;
}

static int ice_create_txq(struct ice_txq *txq, int index)
{
	txq->dpdk_txq = dpdk_data->tx_queues[index];

	return 0;
}

static int ice_qs_init_qs(unsigned int nr_rxq)
{
	int i;
	struct hardware_q *qs[nr_rxq];

	for (i = 0; i < nr_rxq; i++)
		qs[i] = &rxqs[i].rxq;

	return init_qs(qs, nr_rxq, ice_recv_pkts);
}

static void map_page(int id, void *addr)
{
	char path[64];
	int fd;
	void *va;

	snprintf(path, sizeof(path), "/dev/hugepages/rtemap_%d", id);
	fd = open(path, O_CREAT | O_RDWR, 0600);
	va = mmap(addr, 0x200000, PROT_READ | PROT_WRITE,
		MAP_SHARED | MAP_POPULATE | MAP_FIXED, fd, 0);
	if (va == MAP_FAILED) {
		log_err("error, map failed");
		perror("mmap");
	}
	log_info("path: %s, va: %p, addr: %p", path, va, addr);
	*(volatile int *)addr = *(volatile int *)addr;
	close(fd);
}

static void map_device_memory(int id, void *addr, size_t size)
{
	char path[64];
	int fd;
	void *va;

	snprintf(path, sizeof(path), "/sys/bus/pci/devices/0000:af:00.0/resource%d", id);
	fd = open(path, O_RDWR);
	va = mmap(addr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (va == MAP_FAILED) {
		log_err("error, map failed");
		perror("mmap");
	}
	log_info("path: %s, va: %p, addr: %p", path, va, addr);
	*(volatile int *)addr = *(volatile int *)addr;
	close(fd);
}

static struct net_driver_ops ice_net_ops_queue_steering = {
	.rx_batch = qs_gather_rx_no_parity,
	.tx_single = ice_xmit_pkt,
	.steer_flows = qs_steer,
	.register_flow = qs_register_flow,
	.deregister_flow = qs_deregister_flow,
//	.get_flow_affinity = TODO: implement this to support affinitized conns
	.rxq_has_work = qs_have_work_no_parity,
};

/*
 * ice_init - initialize TX/RX queues
 */
int ice_init(struct hardware_q **rxq_out, struct direct_txq **txq_out,
	unsigned int nr_rxq, unsigned int nr_txq, void *rx_buf, size_t rx_len)
{
	void *addr;
	int i, ret;
	size_t nr_pages;

	log_debug("in ice_init with %d %d queues", nr_rxq, nr_txq);

	if (!is_power_of_two(nr_rxq) || nr_rxq > NCPU)
		return -EINVAL;

	/* TEMP: map DPDK memory, there are two chunks */
	addr = (void *) 0x100200000;
	for (i = 0; i < 64; i++) {
		map_page(i, addr);
		addr += 0x200000;
	}
	addr = (void *) 0x1100a00000;
	for (i = 32768; i < 32793; i++) {
		map_page(i, addr);
		addr += 0x200000;
	}

	/* TEMP: map device memory, two chunks */
	map_device_memory(0, (void *) 0x2101000000, 0x8000000);
	map_device_memory(3, (void *) 0x2109000000, 0x10000);

	/* create RX queues */
	for (i = 0; i < nr_rxq; i++) {
		ret = ice_create_rxq(&rxqs[i], i);
		if (ret) {
			log_err("ice_init: failed to create rxq");
			return ret;
		}

		rxq_out[i] = &rxqs[i].rxq;
	}

	/* initialize queue steering for RX queues */
	ret = ice_qs_init_qs(nr_rxq);
	if (ret)
		return ret;

	/* create TX queues */
	for (i = 0; i < nr_txq; i++) {
		ret = ice_create_txq(&txqs[i], i);
		if (ret) {
			log_err("ice_init: failed to create txq");
			return ret;
		}

		txq_out[i] = &txqs[i].txq;
	}

	/* setup virtual->physical address mappings for RX buffers */
	nr_pages = div_up(rx_len, PGSIZE_2MB);
	rx_addrs = aligned_alloc(CACHE_LINE_SIZE, sizeof(struct page_paddrs) +
				sizeof(physaddr_t) * nr_pages);
	if (!rx_addrs)
		return -ENOMEM;
	rx_addrs->base = rx_buf;
	rx_addrs->len = rx_len;
	ret = mem_lookup_page_phys_addrs(rx_addrs->base, rx_addrs->len,
					PGSIZE_2MB, rx_addrs->paddrs);
	if (ret)
		return -1;

	/* setup virtual->physical address mappings for TX buffers */
	nr_pages = div_up(netcfg.tx_region.len, PGSIZE_2MB);
	tx_addrs = aligned_alloc(CACHE_LINE_SIZE, sizeof(struct page_paddrs) +
				sizeof(physaddr_t) * nr_pages);
	if (!tx_addrs)
		return -ENOMEM;
	tx_addrs->base = netcfg.tx_region.base;
	tx_addrs->len = netcfg.tx_region.len;
	ret = mem_lookup_page_phys_addrs(tx_addrs->base, tx_addrs->len,
					PGSIZE_2MB, tx_addrs->paddrs);
	if (ret)
		return -1;

	net_ops = ice_net_ops_queue_steering;

	return 0;
}

#endif
