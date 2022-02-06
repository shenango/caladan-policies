/*
 * hw_timestamp.h - methods for tracking hardware timestamps in MLX5
 */

#ifdef MLX5

#include <infiniband/mlx5dv.h>
#include <util/mmio.h>

#include "defs.h"

extern double device_us_per_cycle;
extern double cpu_cycles_per_device_cycle;
extern uint32_t curr_hw_time;
extern void *hca_core_clock;

static inline bool is_hw_timestamp_enabled()
{
	return !cfg.no_hw_qdel;
}

static inline void hw_timestamp_update(void)
{
	if (cfg.no_hw_qdel)
		return;

	/* read the low 32 bits of the hardware counter */
	curr_hw_time = be32toh(mmio_read32_be(hca_core_clock + 4));
}

static inline uint64_t hw_timestamp_delay_us(struct mlx5_cqe64 *cqe)
{
	double us;
	uint32_t hwstamp = (uint32_t)be64toh(ACCESS_ONCE(cqe->timestamp));

	if (wraps_lte(hwstamp, curr_hw_time)) {
		us = (double)(curr_hw_time - hwstamp) * device_us_per_cycle;
		return us;
	}
	return 0;
}

static inline uint64_t hw_timestamp_delay_cpu_cycles(struct mlx5_cqe64 *cqe)
{
	double cycles;
	uint32_t hwstamp = (uint32_t)be64toh(ACCESS_ONCE(cqe->timestamp));

	if (wraps_lte(hwstamp, curr_hw_time)) {
		cycles = (double)(curr_hw_time - hwstamp) *
			cpu_cycles_per_device_cycle;
		return cycles;
	}
	return 0;
}

#else

struct mlx5_cqe64;

static inline bool is_hw_timestamp_enabled()
{
	return false;
}
static inline void hw_timestamp_update(void) {}
static inline uint64_t hw_timestamp_delay_us(struct mlx5_cqe64 *cqe)
{
	return 0;
}
#endif
