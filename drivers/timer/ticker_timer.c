#include <system_timer.h>
#include <soc.h>

#include "../bluetooth/controller/util/defines.h"
#include "../bluetooth/controller/util/work.h"
#include "../bluetooth/controller/hal/clock.h"
#include "../bluetooth/controller/ll/ticker.h"

#include "../bluetooth/controller/hal/debug.h"

#if defined(CONFIG_SOC_SERIES_NRF51X)
#define NRF5_IRQ_SWI5_IRQn		NRF51_IRQ_SWI5_IRQn
#define NRF5_IRQ_RTC0_IRQn		NRF51_IRQ_RTC0_IRQn
#else /* NRF52 */
#define NRF5_IRQ_SWI5_IRQn		NRF52_IRQ_SWI5_EGU5_IRQn
#define NRF5_IRQ_RTC0_IRQn		NRF52_IRQ_RTC0_IRQn
#endif

static uint8_t ALIGNED(4) _ticker_nodes[1][TICKER_NODE_T_SIZE];
static uint8_t ALIGNED(4) _ticker_users[1][TICKER_USER_T_SIZE];
static uint8_t ALIGNED(4) _ticker_user_ops[3][TICKER_USER_OP_T_SIZE];

static uint32_t clock_accumulated_count;

#ifdef CONFIG_SYS_POWER_MANAGEMENT
#ifdef CONFIG_NANOKERNEL
void _sys_power_save_idle_exit(int32_t ticks)
{
}
#elif defined(CONFIG_TICKLESS_IDLE)
extern int32_t _sys_idle_elapsed_ticks;

static uint8_t _tickless_enter_mutex;
static uint8_t _tickless_exit_mutex;

static void ticker_op_done(uint32_t status, void *context)
{
	ASSERT(status == TICKER_STATUS_SUCCESS);

	*((uint8_t *)context) = 0;
}

void _timer_idle_enter(int32_t ticks)
{
	if (!_tickless_enter_mutex) {
		uint32_t status;
		uint16_t lazy;

		_tickless_enter_mutex = 1;

		if ((ticks < 0) || (ticks >= UINT16_MAX)) {
			lazy = UINT16_MAX;
		} else {
			lazy = ticks + 1;
		}

		ASSERT(lazy);

		status = ticker_update(1, 0, 0, 0, 0, 0, 0, lazy, 0,
				       ticker_op_done, &_tickless_exit_mutex);
		ASSERT((status == TICKER_STATUS_SUCCESS) ||
		       (status == TICKER_STATUS_BUSY));
	}
}

void _timer_idle_exit(void)
{
	if (!_tickless_exit_mutex) {
		uint32_t status;

		_tickless_exit_mutex = 1;

		status = ticker_update(1, 0, 0, 0, 0, 0, 0, 1, 0,
				       ticker_op_done, &_tickless_enter_mutex);
		ASSERT((status == TICKER_STATUS_SUCCESS) ||
		       (status == TICKER_STATUS_BUSY));
	}
}
#endif /* CONFIG_TICKLESS_IDLE */
#endif /* SYS_POWER_MANAGEMENT */

static void ticker_timeout(uint32_t ticks_at_expire, uint32_t remainder,
			   uint16_t lazy, void *context)
{
	ARG_UNUSED(ticks_at_expire);
	ARG_UNUSED(remainder);
	ARG_UNUSED(lazy);
	ARG_UNUSED(context);

	clock_accumulated_count += sys_clock_hw_cycles_per_tick;

#ifdef CONFIG_TICKLESS_IDLE
	_sys_idle_elapsed_ticks = lazy + 1;
#endif

	_sys_clock_tick_announce();
}

static void swi5_nrf5_isr(void *arg)
{
	work_run(NRF5_IRQ_SWI5_IRQn);
}

static void rtc0_nrf5_isr(void *arg)
{
	uint32_t compare0, compare1;

	/* store interested events */
	compare0 = NRF_RTC0->EVENTS_COMPARE[0];
	compare1 = NRF_RTC0->EVENTS_COMPARE[1];

	/* On compare0 run ticker worker instance0 */
	if (compare0) {
		NRF_RTC0->EVENTS_COMPARE[0] = 0;

		ticker_trigger(0);
	}

	/* On compare1 run ticker worker instance1 */
	if (compare1) {
		NRF_RTC0->EVENTS_COMPARE[1] = 0;

		ticker_trigger(1);
	}

	work_run(RTC0_IRQn);
}

int _sys_clock_driver_init(struct device *device)
{
	uint32_t us;
	int retval;

	ARG_UNUSED(device);

	clock_k32src_start(1);

	_ticker_users[0][0] = 3;

	retval = ticker_init(1, 1, &_ticker_nodes[0],
			     1, &_ticker_users[0],
			     3, &_ticker_user_ops[0]);
	if (retval) {
		return -1;
	}

	IRQ_CONNECT(NRF5_IRQ_RTC0_IRQn, 0, rtc0_nrf5_isr, 0, 0);
	IRQ_CONNECT(NRF5_IRQ_SWI5_IRQn, 2, swi5_nrf5_isr, 0, 0);
	irq_enable(NRF5_IRQ_RTC0_IRQn);
	irq_enable(NRF5_IRQ_SWI5_IRQn);

	us = 1000000/CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	retval = ticker_start(1, 0, 0,
			      ticker_ticks_now_get(),
			      TICKER_US_TO_TICKS(us),
			      TICKER_US_TO_TICKS(us),
			      TICKER_REMAINDER(us),
			      0, 0,
			      ticker_timeout, 0,
			      0, 0);
	if ((retval == TICKER_STATUS_SUCCESS) ||
	    (retval == TICKER_STATUS_BUSY)) {
		retval = 0;
	}

	return retval;
}

uint32_t sys_cycle_get_32(void)
{
	return clock_accumulated_count;
}
