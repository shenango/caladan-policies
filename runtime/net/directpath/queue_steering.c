#ifdef DIRECTPATH

#include "defs.h"

static unsigned int nr_rxq;
static struct hardware_q *qs[NCPU];
static unsigned int queue_assignments[NCPU];
static rx_fn rxfn;

static inline void assign_q(unsigned int qidx, unsigned int kidx)
{
	if (queue_assignments[qidx] == kidx)
		return;

	rcu_hlist_del(&qs[qidx]->link);
	rcu_hlist_add_head(&qs[kidx]->head, &qs[qidx]->link);
	queue_assignments[qidx] = kidx;
}

int init_qs(struct hardware_q **rxqs, unsigned int cnt, rx_fn fn)
{
	int i;

	for (i = 0; i < cnt; i++) {
		qs[i] = rxqs[i];
		rcu_hlist_init_head(&rxqs[i]->head);
		spin_lock_init(&rxqs[i]->lock);
		rcu_hlist_add_head(&rxqs[0]->head, &rxqs[i]->link);
	}

	nr_rxq = cnt;
	rxfn = fn;

	return 0;
}

int qs_steer(unsigned int *new_fg_assignment)
{
	int i;
	for (i = 0; i < maxks; i++)
		assign_q(i, new_fg_assignment[i]);

	assert(nr_rxq < maxks * 2);
	for (; i < nr_rxq; i++)
		assign_q(i, new_fg_assignment[i - maxks]);

	return 0;
}

int qs_register_flow(unsigned int affininty, struct trans_entry *e, void **handle_out)
{
	return 0;
}

int qs_deregister_flow(struct trans_entry *e, void *handle)
{
	return 0;
}

int qs_have_work(struct hardware_q *rxq)
{
	struct hardware_q *mrxq;
	struct rcu_hlist_node *node;

	rcu_hlist_for_each(&rxq->head, node, !preempt_enabled()) {
		mrxq = rcu_hlist_entry(node, struct hardware_q, link);
		if (hardware_q_pending(mrxq))
			return true;
	}

	return false;
}

int qs_have_work_no_parity(struct hardware_q *rxq)
{
	struct hardware_q *mrxq;
	struct rcu_hlist_node *node;

	rcu_hlist_for_each(&rxq->head, node, !preempt_enabled()) {
		mrxq = rcu_hlist_entry(node, struct hardware_q, link);
		if (hardware_q_pending_no_parity(mrxq))
			return true;
	}

	return false;
}

int qs_gather_rx(struct hardware_q *rxq, struct mbuf **ms, unsigned int budget)
{
	struct hardware_q *hq;
	struct rcu_hlist_node *node;

	unsigned int pulled = 0;

	rcu_hlist_for_each(&rxq->head, node, !preempt_enabled()) {
		hq = rcu_hlist_entry(node, struct hardware_q, link);

		if (hardware_q_pending(hq) && spin_try_lock(&hq->lock)) {
			pulled += rxfn(hq, ms + pulled, budget - pulled);
			spin_unlock(&hq->lock);
			if (pulled == budget)
				break;
		}
	}
	return pulled;
}

int qs_gather_rx_no_parity(struct hardware_q *rxq, struct mbuf **ms, unsigned int budget)
{
	struct hardware_q *hq;
	struct rcu_hlist_node *node;

	unsigned int pulled = 0;

	rcu_hlist_for_each(&rxq->head, node, !preempt_enabled()) {
		hq = rcu_hlist_entry(node, struct hardware_q, link);

		if (hardware_q_pending_no_parity(hq) && spin_try_lock(&hq->lock)) {
			pulled += rxfn(hq, ms + pulled, budget - pulled);
			spin_unlock(&hq->lock);
			if (pulled == budget)
				break;
		}
	}
	return pulled;
}

#endif

