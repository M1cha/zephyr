/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hdsc_hc32_lptim

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/hdsc_hc32_clock.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "No or too many LPTIM instances enabled in devicetree");

#define LPTIM_BASE             ((mem_addr_t)(DT_INST_REG_ADDR(0)))
#define LPTIM_MODULE_CLOCK_DEV DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(0, module))
#define LPTIM_MODULE_CLOCK_ID  DT_INST_CLOCKS_CELL_BY_NAME(0, module, clk_id)
#define LPTIM_SOURCE_CLOCK_DEV DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(0, src))
#define LPTIM_SOURCE_CLOCK_ID  DT_INST_CLOCKS_CELL_BY_NAME(0, src, clk_id)
#define LPTIM_IRQN             DT_INST_IRQN(0)
#define LPTIM_IRQ_PRIORITY     DT_INST_IRQ(0, priority)

#define REG_CNT  0x00
#define REG_ARR  0x04
#define REG_CR   0x0C
#define REG_IFR  0x10
#define REG_ICLR 0x14

#define CNT_CNT GENMASK(15, 0)

#define ARR_ARR GENMASK(15, 0)

#define CR_TR      BIT(0)
#define CR_MD      BIT(1)
#define CR_CT      BIT(2)
#define CR_TOG_EN  BIT(3)
#define CR_TCK_SEL GENMASK(5, 4)
#define CR_WT_FLAG BIT(7)
#define CR_GATE    BIT(8)
#define CR_GATE_P  BIT(9)
#define CR_IE      BIT(10)

#define IFR_TF BIT(0)

#define ICLR_TFC BIT(0)

#ifdef CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME
static uint32_t cyc_per_tick;
#define CYC_PER_TICK cyc_per_tick
#else
BUILD_ASSERT((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % CONFIG_SYS_CLOCK_TICKS_PER_SEC) == 0);
#define CYC_PER_TICK                                                                            \
	(sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#endif

#define COUNTER_MAX 0xFFFF
#define MAX_TICKS    ((k_ticks_t)(COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES   (MAX_TICKS * CYC_PER_TICK)
/* Since LPTIM might run on a different clock, interrupt issues may arise if the
 * duration isn't at least this many cycles.
 */
#define MIN_CYCLES   4

typedef uint32_t cycle_t;

static struct k_spinlock lock;
static uint16_t arr_next;
static uint16_t arr_current;
static cycle_t cycle_count;
static cycle_t announced_cycles;
static cycle_t announce_at;
static uint32_t overflow_cyc;
static bool uses_arr = true;
static uint32_t counter_value_before_start;

static uint16_t counter_value_raw(void);

static inline bool lptim_synchronized(void)
{
	return !!(sys_read32(LPTIM_BASE + REG_CR) & CR_WT_FLAG);
}

/**
 * @brief Make sure LPTIM is not reading ARR.
 *
 * Since CPU and LPTIM may use different clocks, LPTIM might be reading the ARR
 * register to copy it to CNT after an overflow. Writing to ARR during that
 * time causes undefined behavior.
 *
 * If the timer is currently running, you have to change ARR as soon as
 * possible after this function returns, otherwise LPTIM might attempt a new
 * read already. There is probably no safe way to modify ARR while the timer
 * is running, if the LPTIM clock is faster than the CPU clock. That'd be a
 * weird thing to do anyway.
 */
static inline void lptim_synchronize(void)
{
	while (!lptim_synchronized()) {
	}
}

static uint32_t timeout_from_arr(uint32_t arr)
{
	return COUNTER_MAX - arr + 1;
}

/** Returns the raw CNT register value. */
static uint16_t counter_value_raw_unsafe(void)
{
	uint32_t count = sys_read32(LPTIM_BASE + REG_CNT);
	return FIELD_GET(CNT_CNT, count);
}

/** Returns the raw CNT register value. */
static uint16_t counter_value_raw(void)
{
	uint16_t value_prev_read;

	/* Since CPU and LPTIM may run on different clocks, the CNT register might be
	 * written to, while we're reading it. So try again until we have read the
	 * same value twice.
	 */
	uint16_t value = counter_value_raw_unsafe();
	do {
		value_prev_read = value;
		value = counter_value_raw_unsafe();
	} while (value != value_prev_read);

	return value;
}

static void clear_isr(void)
{
	uint32_t iclr = sys_read32(LPTIM_BASE + REG_ICLR);
	iclr &= ~ICLR_TFC;
	sys_write32(iclr, LPTIM_BASE + REG_ICLR);
}

static uint32_t elapsed(void)
{
	uint32_t count_raw = counter_value_raw();
	uint32_t arr = uses_arr ? arr_current : 0;
	uint32_t count;

	/* Handling the interrupt here allows handling overflow even if CPU interrupts
	 * are disabled for an extended period of time. Every call to
	 * sys_clock_cycle_get_32, sys_clock_cycle_get_64, sys_clock_elapsed or
	 * sys_clock_set_timeout uses this function internally, and will thus process
	 * the overflow interrupt.
	 */
	uint32_t ifr = sys_read32(LPTIM_BASE + REG_IFR);
	if (ifr & IFR_TF) {
		clear_isr();

		overflow_cyc += timeout_from_arr(arr);

		/* The counter might still be at COUNTER_MAX, depending on how
		 * fast we reacted to the interrupt.
		 */
		count = 0;

		/* We don't use reload mode, so the counter starts at 0 now. */
		uses_arr = false;
	} else if (!lptim_synchronized()) {
		/* The CNT register might not have been initialized with the
		 * value from ARR yet, so using it would yield incorrect timestamps.
		 */
		count = 0;
	} else if (count_raw == counter_value_before_start) {
		/* We're still at the same count we had during (re-)start, so
		 * the CNT register might not have been initialized, yet.
		 */
		count = 0;
	} else {
		__ASSERT_NO_MSG(count_raw >= arr);
		count = count_raw - arr;
	}

	return overflow_cyc + count;
}

/**
 * @brief Set ARR register value.
 *
 * You probably want to call this, while the timer is disabled. Otherwise, the
 * new ARR value won't take effect until the next overflow.
 */
static void set_arr(uint16_t value)
{
	uint32_t arr = sys_read32(LPTIM_BASE + REG_ARR);
	arr &= ~ARR_ARR;
	arr |= FIELD_PREP(ARR_ARR, value);
	sys_write32(arr, LPTIM_BASE + REG_ARR);

	arr_next = value;
}

/**
 * @brief Set the counter interval.
 *
 * Takes the number of cycles to wait, and calls set_arr internally with the
 * correct register value.
 */
static void set_timeout(uint16_t cycles)
{
	__ASSERT(cycles > 0, "LPTIM can't count 0 cycles");
	__ASSERT(cycles <= COUNTER_MAX, "LPTIM is limited to 16 bits");
	set_arr(COUNTER_MAX - cycles);
}

static void hc32_lptim_timer_isr(const void *arg)
{
	uint32_t dcycles;
	uint32_t dticks;

	ARG_UNUSED(arg);

	elapsed();

	cycle_count += overflow_cyc;
	overflow_cyc = 0;

	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		// TODO: this check might be an issue
		uint32_t arr = uses_arr ? arr_current : 0;

		/* LPTIM supports 16bit only and generates an interrupt at least every 2s.
		 * Additionally, we loose accuracy every time sys_clock_set_timeout is called,
		 * because we have to disable the timer for a short time.
		 * To reduce both effects, we let the timer roll over, if we know the kernel
		 * doesn't have to wake up, yet. We won't have to restart the timer and the
		 * CPU can go to sleep again ASAP.
		 */
		if (announce_at > cycle_count &&
				announce_at - cycle_count > timeout_from_arr(arr)) {
			return;
		}

		dcycles = cycle_count - announced_cycles;
		dticks = dcycles / CYC_PER_TICK;
		announced_cycles += dticks * CYC_PER_TICK;
		sys_clock_announce(dticks);

	} else {
		sys_clock_announce(1);
	}
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	/* We don't really want to disable the timer in idle, because we'd loose
	 * track of time.
	 */
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	ticks = (ticks == K_TICKS_FOREVER) ? INT32_MAX : ticks;

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t pending = elapsed();

	/* While calculating the timeout and synchronizing with LPTIM,
	 * additional time may pass. To make the cycle count more accurate at
	 * the cost of making sleeps less accurate, we increase the count later,
	 * with a more recent value.
	 */
	uint32_t cycle_count_local = cycle_count + pending;

	uint32_t cycles;
	uint32_t unannounced = cycle_count - announced_cycles;
	if ((int32_t)unannounced < 0) {
		/* We haven't announced for more than half the 32-bit
		 * wrap duration, because new timeouts keep being set
		 * before the existing one fires.  Force an announce
		 * to avoid loss of a wrap event, making sure that
		 * cycles is at least the minimum cycles possible.
		 *
		 * NOTE: We are worried about wrapping the internal 32-bit cycle counter
		 *       here, NOT the 16bit CNT register.
		 */
		cycles = MIN_CYCLES;
		announce_at = cycle_count_local + cycles;
	} else {
		/* Desired cycles in the future */
		cycles = ((uint32_t)ticks) * CYC_PER_TICK;

		/* Round cycles up to next tick boundary.
		 * The provided `ticks` are relative to  the CURRENT time, which includes
		 * `unannounced`. But since LPTIM doesn't have a compare register, the
		 * `cycles` we set are relative anyway and we have to subtract it again after
		 * rounding.
		 */
		cycles += unannounced;
		cycles = DIV_ROUND_UP(cycles, CYC_PER_TICK) * CYC_PER_TICK;
		cycles -= unannounced;

		announce_at = cycle_count_local + cycles;
		cycles = CLAMP(cycles, MIN_CYCLES, MAX_CYCLES);
	}

	uint32_t cr = sys_read32(LPTIM_BASE + REG_CR);
	bool was_enabled = cr & CR_TR;

	/* The next iteration of the timer will be with MAX_CYCLES anyway, so
	 * just skip changing anything to prevent having to restart the timer
	 * too often.
	 */
	if (was_enabled && cycles >= MAX_CYCLES) {
		k_spin_unlock(&lock, key);
		return;
	}

	/* If we disable and re-enable the timer after this point, we'll get an
	 * overflow interrupt for the old configuration anyway. That would cause
	 * our interrupt handler to add arr_current to the current time, leading
	 * to huge, wrong jumps.
	 *
	 * We prevent running into that situation by only changing the timeout
	 * and not restarting the timer. This might cause sleeps to be a little
	 * less accurate, but the cycle counter will be more accurate.
	 */
	if (was_enabled && counter_value_raw() == COUNTER_MAX - MIN_CYCLES + 1) {
		set_timeout(cycles);
		arr_current = arr_next;
		k_spin_unlock(&lock, key);
		return;
	}

	cr &= ~CR_IE;
	cr &= ~CR_TR;
	sys_write32(cr, LPTIM_BASE + REG_CR);

	lptim_synchronize();

	/* We don't synchronize afterwards, so the CPU can sleep while it's
	 * doing that. By the time the ISR happens, we should be synchronized
	 * already and elapsed() can handle the case where it's not.
	 */
	set_timeout(cycles);

	uint32_t counter_raw = counter_value_raw();
	uint32_t arr = uses_arr ? arr_current : 0;

	cr |= CR_TR;
	cr |= CR_IE;
	sys_write32(cr, LPTIM_BASE + REG_CR);

	cycle_count += overflow_cyc + counter_raw - arr;
	overflow_cyc = 0;

	uses_arr = true;
	arr_current = arr_next;
	counter_value_before_start = counter_raw;

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	uint32_t unannounced = cycle_count - announced_cycles;
	uint32_t delta_cycles = elapsed() + unannounced;
	uint32_t delta_ticks = delta_cycles / CYC_PER_TICK;

	return delta_ticks;
}

void sys_clock_disable(void)
{
	uint32_t cr = sys_read32(LPTIM_BASE + REG_CR);
	cr &= ~CR_IE;
	cr &= ~CR_TR;
	sys_write32(cr, LPTIM_BASE + REG_CR);

	irq_disable(LPTIM_IRQN);
	NVIC_ClearPendingIRQ(LPTIM_IRQN);
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = cycle_count;

	ret += elapsed();
	k_spin_unlock(&lock, key);
	return ret;
}

static int sys_clock_driver_init(void)
{
	uint16_t clock_id;
	int ret;

	if (!device_is_ready(LPTIM_MODULE_CLOCK_DEV)) {
		return -ENODEV;
	}

	if (!device_is_ready(LPTIM_SOURCE_CLOCK_DEV)) {
		return -ENODEV;
	}

	uint8_t tck_sel;
	switch (LPTIM_SOURCE_CLOCK_ID) {
	case HDSC_HC32_CLKID_PERI:
		tck_sel = 0;
		break;
	case HDSC_HC32_CLKID_XTL:
		tck_sel = 2;
		break;
	case HDSC_HC32_CLKID_RCL:
		tck_sel = 3;
		break;
	default:
		return -ENOTSUP;
	}

	clock_id = LPTIM_MODULE_CLOCK_ID;
	ret = clock_control_on(LPTIM_MODULE_CLOCK_DEV, &clock_id);
	if (ret) {
		return ret;
	}

	clock_id = LPTIM_SOURCE_CLOCK_ID;
	uint32_t frequency;
	ret = clock_control_get_rate(LPTIM_SOURCE_CLOCK_DEV, &clock_id, &frequency);
	if (ret) {
		return ret;
	}
	if (frequency > INT_MAX) {
		return -ERANGE;
	}

#ifdef CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME
	extern int z_clock_hw_cycles_per_sec;
	z_clock_hw_cycles_per_sec = frequency;

	__ASSERT_NO_MSG((z_clock_hw_cycles_per_sec % CONFIG_SYS_CLOCK_TICKS_PER_SEC) == 0);

	cyc_per_tick =
		sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
#endif

	IRQ_CONNECT(LPTIM_IRQN, LPTIM_IRQ_PRIORITY, hc32_lptim_timer_isr, NULL, 0);

	uint32_t cr = sys_read32(LPTIM_BASE + REG_CR);
	cr &= ~CR_TR;
	/* We use counting mode so the timer starts at 0 after the overflow
	 * interrupt. This has the following advantages:
	 * - We have more time to handle the interrupt and set a new timeout.
	 *   This becomes increasingly important if the previous timeout was
	 *   very small.
	 * - Zephyr likes to set the timeout to K_FOREVER before setting the new
	 *   timeout. If we start counting from 0, the timer is already doing
	 *   that and we don't have to restart it.
	 */
	cr &= ~CR_MD;
	cr &= ~CR_CT;
	cr &= ~CR_TOG_EN;

	cr &= ~CR_TCK_SEL;
	cr |= FIELD_PREP(CR_TCK_SEL, tck_sel);

	cr &= ~CR_GATE;
	cr &= ~CR_GATE_P;
	cr &= ~CR_IE;

	sys_write32(cr, LPTIM_BASE + REG_CR);

	announce_at = 0;
	lptim_synchronize();
	set_timeout(MAX(CYC_PER_TICK, MIN_CYCLES));
	arr_current = arr_next;

	clear_isr();
	NVIC_ClearPendingIRQ(LPTIM_IRQN);

	cr |= CR_TR;
	cr |= CR_IE;
	sys_write32(cr, LPTIM_BASE + REG_CR);
	uses_arr = true;

	irq_enable(LPTIM_IRQN);

	return 0;
}
SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
