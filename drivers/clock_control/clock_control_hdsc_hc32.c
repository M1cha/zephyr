#define DT_DRV_COMPAT hdsc_hc32_sysctrl

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/hdsc_hc32_clock.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_hdsc_hc32, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define REG_SYSCTRL0   0x00
#define REG_SYSCTRL1   0x04
#define REG_SYSCTRL2   0x08
#define REG_RCH_CR     0x0C
#define REG_XTH_CR     0x10
#define REG_RCL_CR     0x14
#define REG_XTL_CR     0x18
#define REG_PERI_CLKEN 0x20
#define REG_PLL_CR     0x3C

#define SYSCTRL0_RCH_EN     BIT(0)
#define SYSCTRL0_XTH_EN     BIT(1)
#define SYSCTRL0_RCL_EN     BIT(2)
#define SYSCTRL0_XTL_EN     BIT(3)
#define SYSCTRL0_PLL_EN     BIT(4)
#define SYSCTRL0_SYSCLK_SRC GENMASK(7, 5)
#define SYSCTRL0_HCLK_PRS   GENMASK(10, 8)
#define SYSCTRL0_PCLK_PRS   GENMASK(12, 11)

#define RCL_CR_TRIM    GENMASK(9, 0)
#define RCL_CR_STARTUP GENMASK(11, 10)
#define RCL_CR_STABLE  BIT(12)

#define RCH_CR_TRIM   GENMASK(10, 0)
#define RCH_CR_STABLE BIT(11)

#define XTL_CR_DRIVE   GENMASK(1, 0)
#define XTL_CR_AMP     GENMASK(3, 2)
#define XTL_CR_STARTUP GENMASK(5, 4)
#define XTL_CR_STABLE  BIT(6)

#define XTH_CR_DRIVE   GENMASK(1, 0)
#define XTH_CR_FSEL    GENMASK(3, 2)
#define XTH_CR_STARTUP GENMASK(5, 4)
#define XTH_CR_STABLE  BIT(6)

#define SYSCTRL2_SYSCTRL2 GENMASK(15, 0)

#define PERI_CLKEN_GPIO BIT(28)

#define CAL_RCL_38400  sys_read16(0x00100C20)
#define CAL_RCL_32768  sys_read16(0x00100C22)
#define CAL_RCH_4M     sys_read16(0x00100C08)
#define CAL_RCH_8M     sys_read16(0x00100C06)
#define CAL_RCH_16M    sys_read16(0x00100C04)
#define CAL_RCH_22_12M sys_read16(0x00100C02)
#define CAL_RCH_24M    sys_read16(0x00100C00)

#define GPIO_BASE      ((mem_addr_t)0x40020C00)
#define GPIO_REG_PCADS 0x1AC
#define GPIO_REG_PDADS 0x1CC

#define FLASH_BASE         ((mem_addr_t)0x40020000)
#define FLASH_REG_CR       0x20
#define FLASH_REG_BYPASS   0x2C
#define FLASH_CR_WAIT      GENMASK(3, 2)
#define FLASH_BYPASS_BYSEQ GENMASK(15, 0)

#define CLK_SRC_IS(clk, src)                                                                       \
	DT_SAME_NODE(DT_CLOCKS_CTLR_BY_IDX(DT_INST_CLOCKS_CTLR_BY_NAME(0, clk), 0),                \
		     DT_INST_CLOCKS_CTLR_BY_NAME(0, src))

struct hdsc_hc32_rcl_config {
	bool enabled;
	uint16_t startup_time_cycles;
	uint32_t rate;
};

struct hdsc_hc32_rch_config {
	bool enabled;
	uint32_t rate;
};

struct hdsc_hc32_xtl_config {
	bool enabled;
	uint32_t rate;
	uint8_t amplitude;
	uint8_t drive;
	uint16_t startup_time_cycles;
};

struct hdsc_hc32_xth_config {
	bool enabled;
	uint32_t rate;
	uint8_t drive;
	uint16_t startup_time_cycles;
};

struct hdsc_hc32_sys_config {
	uint32_t div;
	uint32_t source;
};

struct hdsc_hc32_peri_config {
	uint32_t div;
};

struct clock_control_hdsc_hc32_config {
	mem_addr_t base;

	struct hdsc_hc32_rcl_config rcl;
	struct hdsc_hc32_rch_config rch;
	struct hdsc_hc32_xtl_config xtl;
	struct hdsc_hc32_xth_config xth;

	struct hdsc_hc32_sys_config sys;
	struct hdsc_hc32_peri_config peri;
};

struct clock_control_hdsc_hc32_data {
	struct k_spinlock lock;
};

static void write_flash_bypass(uint16_t value)
{
	uint32_t bypass = sys_read32(FLASH_BASE + FLASH_REG_BYPASS);
	bypass &= ~FLASH_BYPASS_BYSEQ;
	bypass |= FIELD_PREP(FLASH_BYPASS_BYSEQ, value);
	sys_write32(bypass, FLASH_BASE + FLASH_REG_BYPASS);
}

static void flash_unlock(void)
{
	write_flash_bypass(0x5A5A);
	write_flash_bypass(0xA5A5);
}

static void write_sysctrl2(const struct device *dev, uint16_t value)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;

	uint32_t sysctrl2 = sys_read32(config->base + REG_SYSCTRL2);
	sysctrl2 &= ~SYSCTRL2_SYSCTRL2;
	sysctrl2 |= FIELD_PREP(SYSCTRL2_SYSCTRL2, value);
	sys_write32(sysctrl2, config->base + REG_SYSCTRL2);
}

static void unlock(const struct device *dev)
{
	write_sysctrl2(dev, 0x5A5A);
	write_sysctrl2(dev, 0xA5A5);
}

static void enable_gpio_clock(const struct device *dev)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;

	uint32_t value = sys_read32(config->base + REG_PERI_CLKEN);
	value |= PERI_CLKEN_GPIO;
	sys_write32(value, config->base + REG_PERI_CLKEN);
}

static int get_clk_frequency(const struct device *dev, uint32_t id, uint32_t *frequency_ret)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	uint32_t reg;
	uint32_t trim;
	uint32_t frequency;

	switch (id) {
	case HDSC_HC32_CLKID_RCH:
		reg = sys_read32(config->base + REG_RCH_CR);

		trim = FIELD_GET(RCH_CR_TRIM, reg);
		if (trim == CAL_RCH_4M) {
			frequency = 4000000;
		} else if (trim == CAL_RCH_8M) {
			frequency = 8000000;
		} else if (trim == CAL_RCH_16M) {
			frequency = 16000000;
		} else if (trim == CAL_RCH_22_12M) {
			frequency = 22120000;
		} else if (trim == CAL_RCH_24M) {
			frequency = 24000000;
		} else {
			frequency = 4000000;
		}
		break;
	case HDSC_HC32_CLKID_RCL:
		reg = sys_read32(config->base + REG_RCL_CR);

		trim = FIELD_GET(RCL_CR_TRIM, reg);
		if (trim == CAL_RCL_32768) {
			frequency = 32768;
		} else if (trim == CAL_RCL_38400) {
			frequency = 38400;
		} else {
			frequency = 32768;
		}
		break;
	case HDSC_HC32_CLKID_XTL:
		if (!config->xtl.enabled) {
			return -ENOENT;
		}
		frequency = config->xtl.rate;
		break;
	case HDSC_HC32_CLKID_XTH:
		if (!config->xth.enabled) {
			return -ENOENT;
		}
		frequency = config->xth.rate;
		break;
	default:
		return -EINVAL;
	}

	*frequency_ret = frequency;

	return 0;
}

static int get_clk_frequency_derived(const struct device *dev, uint32_t id, uint32_t *frequency_ret)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	int ret;

	if (id <= HDSC_HC32_CLKID_PLL) {
		return get_clk_frequency(dev, id, frequency_ret);
	}

	uint32_t sysctrl0 = sys_read32(config->base + REG_SYSCTRL0);

	if (id == HDSC_HC32_CLKID_SYS) {
		uint32_t source = FIELD_GET(SYSCTRL0_SYSCLK_SRC, sysctrl0);
		ret = get_clk_frequency(dev, source, frequency_ret);
		if (ret) {
			return ret;
		}

		*frequency_ret = *frequency_ret >> FIELD_GET(SYSCTRL0_HCLK_PRS, sysctrl0);
		return 0;
	}

	if (id == HDSC_HC32_CLKID_PERI) {
		ret = get_clk_frequency_derived(dev, HDSC_HC32_CLKID_SYS, frequency_ret);
		if (ret) {
			return ret;
		}

		*frequency_ret = *frequency_ret >> FIELD_GET(SYSCTRL0_PCLK_PRS, sysctrl0);
		return 0;
	}

	if (id >= HDSC_HC32_CLKID_MODULE_BASE) {
		return get_clk_frequency_derived(dev, HDSC_HC32_CLKID_PERI, frequency_ret);
	}

	return -ENOENT;
}

static int set_sys_clk_source(const struct device *dev, uint32_t source)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	int ret;
	uint32_t value;

	uint32_t frequency;
	ret = get_clk_frequency(dev, source, &frequency);
	if (ret) {
		return ret;
	}

	value = sys_read32(config->base + REG_SYSCTRL0);
	frequency >>= FIELD_GET(SYSCTRL0_HCLK_PRS, value);

	uint8_t flash_wait_new;
	switch (frequency) {
	case 0 ... 24000000:
		flash_wait_new = 0;
		break;
	case 24000001 ... 47999999:
		flash_wait_new = 1;
		break;
	default:
		flash_wait_new = 2;
		break;
	}

	uint32_t flash_cr = sys_read32(FLASH_BASE + FLASH_REG_CR);
	uint8_t flash_wait_old = FIELD_GET(FLASH_CR_WAIT, flash_cr);

	flash_cr &= ~FLASH_CR_WAIT;
	flash_cr |= FIELD_PREP(FLASH_CR_WAIT, flash_wait_new);

	/* Increase wait time before switching to the higher frequency, so flash reads
	 * will keep working afterwards.
	 */
	if (flash_wait_new > flash_wait_old) {
		flash_unlock();
		sys_write32(flash_cr, FLASH_BASE + FLASH_REG_CR);
	}

	unlock(dev);
	value = sys_read32(config->base + REG_SYSCTRL0);
	value &= ~SYSCTRL0_SYSCLK_SRC;
	value |= FIELD_PREP(SYSCTRL0_SYSCLK_SRC, source);
	sys_write32(value, config->base + REG_SYSCTRL0);

	extern int z_clock_hw_cycles_per_sec;
	z_clock_hw_cycles_per_sec = frequency;

	/* Decrease wait time after switching to lower frequency, because doing that
	 * before the switch would break flash reads.
	 */
	if (flash_wait_new < flash_wait_old) {
		flash_unlock();
		sys_write32(flash_cr, FLASH_BASE + FLASH_REG_CR);
	}

	return 0;
}

static int init_rcl(const struct device *dev, uint16_t startup_time_cycles, uint32_t rate)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	uint32_t value;

	uint8_t startup;
	switch (startup_time_cycles) {
	case 4:
		startup = 0;
		break;
	case 16:
		startup = 1;
		break;
	case 64:
		startup = 2;
		break;
	case 256:
		startup = 3;
		break;
	default:
		return -EINVAL;
	}

	uint32_t trim;
	switch (config->rcl.rate) {
	case 38400:
		trim = CAL_RCL_38400;
		break;
	case 32768:
		trim = CAL_RCL_32768;
		break;
	default:
		return -EINVAL;
	}

	if (trim > RCL_CR_TRIM) {
		return -EINVAL;
	}

	value = sys_read32(config->base + REG_RCL_CR);
	value &= ~RCL_CR_TRIM;
	value |= FIELD_PREP(RCL_CR_TRIM, trim);
	value &= ~RCL_CR_STARTUP;
	value |= FIELD_PREP(RCL_CR_STARTUP, startup);
	sys_write32(value, config->base + REG_RCL_CR);

	unlock(dev);
	value = sys_read32(config->base + REG_SYSCTRL0);
	value |= SYSCTRL0_RCL_EN;
	sys_write32(value, config->base + REG_SYSCTRL0);

	while (!(sys_read32(config->base + REG_RCL_CR) & RCL_CR_STABLE))
		;

	return 0;
}

static int init_rch(const struct device *dev)
{
	/* NOTE: Even if the target frequency is the default 4M, we still want to go
	 * through this whole process to apply the calibration value.
	 */

	const struct clock_control_hdsc_hc32_config *config = dev->config;
	int ret;
	uint32_t value;

	uint32_t trim;
	switch (config->rch.rate) {
	case 4000000:
		trim = CAL_RCH_4M;
		break;
	case 8000000:
		trim = CAL_RCH_8M;
		break;
	case 16000000:
		trim = CAL_RCH_16M;
		break;
	case 22120000:
		trim = CAL_RCH_22_12M;
		break;
	case 24000000:
		trim = CAL_RCH_24M;
		break;
	default:
		return -EINVAL;
	}

	if (trim > RCH_CR_TRIM) {
		return -EINVAL;
	}

	/* Init and RCL and switch to it, so we can change the speed of the RCL, which
	 * we are currently running on.
	 */

	ret = init_rcl(dev, 256, 32768);
	if (ret) {
		return ret;
	}

	ret = set_sys_clk_source(dev, HDSC_HC32_CLKID_RCL);
	if (ret) {
		return ret;
	}

	value = sys_read32(config->base + REG_RCH_CR);
	value &= ~RCH_CR_TRIM;
	value |= FIELD_PREP(RCH_CR_TRIM, trim);
	sys_write32(value, config->base + REG_RCH_CR);

	unlock(dev);
	value = sys_read32(config->base + REG_SYSCTRL0);
	value |= SYSCTRL0_RCH_EN;
	sys_write32(value, config->base + REG_SYSCTRL0);

	while (!(sys_read32(config->base + REG_RCH_CR) & RCH_CR_STABLE))
		;

	/* Switch back to RCH.
	 * Even if that's not the clock we want to be running with at the end, it's
	 * still worth doing that, because the RCL is quite slow.
	 */

	ret = set_sys_clk_source(dev, HDSC_HC32_CLKID_RCH);
	if (ret) {
		return ret;
	}

	unlock(dev);
	value = sys_read32(config->base + REG_SYSCTRL0);
	value &= ~SYSCTRL0_RCL_EN;
	sys_write32(value, config->base + REG_SYSCTRL0);

	return 0;
}

static int init_xtl(const struct device *dev)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;

	uint8_t startup;
	switch (config->xtl.startup_time_cycles) {
	case 256:
		startup = 0;
		break;
	case 1024:
		startup = 1;
		break;
	case 4096:
		startup = 2;
		break;
	case 16384:
		startup = 3;
		break;
	default:
		return -EINVAL;
	}

	enable_gpio_clock(dev);

	/* Put PC14 and PC15 in analog mode. */
	uint32_t value = sys_read32(GPIO_BASE + GPIO_REG_PCADS);
	value |= BIT(14);
	value |= BIT(15);
	sys_write32(value, GPIO_BASE + GPIO_REG_PCADS);

	value = sys_read32(config->base + REG_XTL_CR);
	value &= ~XTL_CR_DRIVE;
	value |= FIELD_PREP(XTL_CR_DRIVE, config->xtl.drive);
	value &= ~XTL_CR_AMP;
	value |= FIELD_PREP(XTL_CR_AMP, config->xtl.amplitude);
	value &= ~XTL_CR_STARTUP;
	value |= FIELD_PREP(XTL_CR_STARTUP, startup);
	sys_write32(value, config->base + REG_XTL_CR);

	unlock(dev);
	value = sys_read32(config->base + REG_SYSCTRL0);
	value |= SYSCTRL0_XTL_EN;
	sys_write32(value, config->base + REG_SYSCTRL0);

	while (!(sys_read32(config->base + REG_XTL_CR) & XTL_CR_STABLE))
		;

	return 0;
}

static int init_xth(const struct device *dev)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;

	uint8_t fsel;
	switch (config->xth.rate) {
	case 4000000 ... 7999999:
		fsel = 0;
		break;
	case 8000000 ... 15999999:
		fsel = 1;
		break;
	case 16000000 ... 23999999:
		fsel = 2;
		break;
	case 24000000 ... 32000000:
		fsel = 3;
		break;
	default:
		return -EINVAL;
	}

	uint8_t startup;
	switch (config->xth.startup_time_cycles) {
	case 256:
		startup = 0;
		break;
	case 1024:
		startup = 1;
		break;
	case 4096:
		startup = 2;
		break;
	case 16384:
		startup = 3;
		break;
	default:
		return -EINVAL;
	}

	enable_gpio_clock(dev);

	/* Put PD00 and PD01 in analog mode. */
	uint32_t value = sys_read32(GPIO_BASE + GPIO_REG_PDADS);
	value |= BIT(0);
	value |= BIT(1);
	sys_write32(value, GPIO_BASE + GPIO_REG_PDADS);

	value = sys_read32(config->base + REG_XTH_CR);
	value &= ~XTH_CR_DRIVE;
	value |= FIELD_PREP(XTH_CR_DRIVE, config->xth.drive);
	value &= ~XTH_CR_FSEL;
	value |= FIELD_PREP(XTH_CR_FSEL, fsel);
	value &= ~XTH_CR_STARTUP;
	value |= FIELD_PREP(XTH_CR_STARTUP, startup);
	sys_write32(value, config->base + REG_XTH_CR);

	unlock(dev);
	value = sys_read32(config->base + REG_SYSCTRL0);
	value |= SYSCTRL0_XTH_EN;
	sys_write32(value, config->base + REG_SYSCTRL0);

	while (!(sys_read32(config->base + REG_XTH_CR) & XTH_CR_STABLE))
		;

	return 0;
}

static int clock_control_hdsc_hc32_init(const struct device *dev)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	int ret;

	LOG_DBG("Start clock init");
	/*LOG_DBG("CAL_RCL_32768 = %u", CAL_RCL_32768);
	LOG_DBG("CAL_RCL_38400 = %u", CAL_RCL_38400);
	LOG_DBG("CAL_RCH_4M = %u", CAL_RCH_4M);
	LOG_DBG("CAL_RCH_8M = %u", CAL_RCH_8M);
	LOG_DBG("CAL_RCH_16M = %u", CAL_RCH_16M);
	LOG_DBG("CAL_RCH_22.12M = %u", CAL_RCH_22_12M);
	LOG_DBG("CAL_RCH_24M = %u", CAL_RCH_24M);*/

	/* This MUST be first, because it re-configures the RCL. */
	if (config->rch.enabled) {
		LOG_DBG("Init RCH");
		ret = init_rch(dev);
		if (ret) {
			return ret;
		}
	}

	if (config->rcl.enabled) {
		LOG_DBG("Init RCL");
		ret = init_rcl(dev, config->rcl.startup_time_cycles, config->rcl.rate);
		if (ret) {
			return ret;
		}
	}

	if (config->xtl.enabled) {
		LOG_DBG("Init XTL");
		ret = init_xtl(dev);
		if (ret) {
			return ret;
		}
	}

	if (config->xth.enabled) {
		LOG_DBG("Init XTH");
		ret = init_xth(dev);
		if (ret) {
			return ret;
		}
	}

	if (config->sys.source != HDSC_HC32_CLKID_RCH) {
		ret = set_sys_clk_source(dev, config->sys.source);
		if (ret) {
			return ret;
		}

		if (!config->rch.enabled) {
			LOG_DBG("Disable RCH");
			unlock(dev);
			uint32_t value = sys_read32(config->base + REG_SYSCTRL0);
			value &= ~SYSCTRL0_RCH_EN;
			sys_write32(value, config->base + REG_SYSCTRL0);
		}
	}

	LOG_DBG("Clock init done");

	return 0;
}

static int clock_control_hc32_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	struct clock_control_hdsc_hc32_data *data = dev->data;
	uint8_t id = *(uint8_t *)sys;

	if (id < HDSC_HC32_CLKID_MODULE_BASE) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t value = sys_read32(config->base + REG_PERI_CLKEN);
	value |= BIT(id - HDSC_HC32_CLKID_MODULE_BASE);
	sys_write32(value, config->base + REG_PERI_CLKEN);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int clock_control_hc32_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	struct clock_control_hdsc_hc32_data *data = dev->data;
	uint8_t id = *(uint8_t *)sys;

	if (id < HDSC_HC32_CLKID_MODULE_BASE) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t value = sys_read32(config->base + REG_PERI_CLKEN);
	value &= ~BIT(id - HDSC_HC32_CLKID_MODULE_BASE);
	sys_write32(value, config->base + REG_PERI_CLKEN);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int clock_control_hc32_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *rate)
{
	uint8_t id = *(uint8_t *)sys;
	return get_clk_frequency_derived(dev, id, rate);
}

static enum clock_control_status clock_control_hc32_get_status(const struct device *dev,
							       clock_control_subsys_t sys)
{
	const struct clock_control_hdsc_hc32_config *config = dev->config;
	uint8_t id = *(uint8_t *)sys;
	uint32_t sysctrl0 = sys_read32(config->base + REG_SYSCTRL0);
	bool enabled = false;

	switch (id) {
	case HDSC_HC32_CLKID_RCH:
		enabled = sysctrl0 & SYSCTRL0_RCH_EN;
		break;
	case HDSC_HC32_CLKID_XTH:
		enabled = sysctrl0 & SYSCTRL0_XTH_EN;
		break;
	case HDSC_HC32_CLKID_RCL:
		enabled = sysctrl0 & SYSCTRL0_RCL_EN;
		break;
	case HDSC_HC32_CLKID_XTL:
		enabled = sysctrl0 & SYSCTRL0_XTL_EN;
		break;
	case HDSC_HC32_CLKID_PLL:
		enabled = sysctrl0 & SYSCTRL0_PLL_EN;
		break;
	case HDSC_HC32_CLKID_SYS:
	case HDSC_HC32_CLKID_PERI:
		enabled = true;
		break;
	default:
		/* They derive from the peripheral clock which is always enabeld. */
		enabled = id >= HDSC_HC32_CLKID_MODULE_BASE;
		break;
	}

	return enabled ? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static const struct clock_control_driver_api clock_control_hdsc_hc32_api = {
	.on = clock_control_hc32_on,
	.off = clock_control_hc32_off,
	.get_rate = clock_control_hc32_get_rate,
	.get_status = clock_control_hc32_get_status,
};

BUILD_ASSERT(!CLK_SRC_IS(sys, rcl) || DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, rcl), okay),
	     "RCL is disabled, but needed for system clock");
BUILD_ASSERT(!CLK_SRC_IS(sys, rch) || DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, rch), okay),
	     "RCH is disabled, but needed for system clock");
BUILD_ASSERT(!CLK_SRC_IS(sys, xtl) || DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), okay),
	     "XTL is disabled, but needed for system clock");
BUILD_ASSERT(!CLK_SRC_IS(sys, xth) || DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, xth), okay),
	     "XTH is disabled, but needed for system clock");

static const struct clock_control_hdsc_hc32_config clock_control_hdsc_hc32_config = {
	.base = DT_INST_REG_ADDR(0),
	.rcl =
		{
			.enabled = DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, rcl), okay),
#if DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, rcl), okay)
			.startup_time_cycles =
				DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, rcl), startup_time_cycles),
			.rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, rcl), clock_frequency),
#endif
		},
	.rch =
		{
			.enabled = DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, rch), okay),
#if DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, rch), okay)
			.rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, rch), clock_frequency),
#endif
		},
	.xtl =
		{
			.enabled = DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), okay),
#if DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), okay)
			.rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), clock_frequency),
			.amplitude = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), amplitude),
			.drive = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), drive),
			.startup_time_cycles =
				DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtl), startup_time_cycles),
#endif
		},
	.xth =
		{
			.enabled = DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, xth), okay),
#if DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, xth), okay)
			.rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xth), clock_frequency),
			.drive = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xth), drive),
			.startup_time_cycles =
				DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xth), startup_time_cycles),
#endif
		},
	.sys =
		{
			.div = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, sys), clock_div),
#if CLK_SRC_IS(sys, rcl)
			.source = HDSC_HC32_CLKID_RCL,
#elif CLK_SRC_IS(sys, rch)
			.source = HDSC_HC32_CLKID_RCH,
#elif CLK_SRC_IS(sys, xtl)
			.source = HDSC_HC32_CLKID_XTL,
#elif CLK_SRC_IS(sys, xth)
			.source = HDSC_HC32_CLKID_XTH,
#else
#error "Unsupported sys-clk source"
#endif
		},
	.peri =
		{
			.div = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, peri), clock_div),
		},
};

static struct clock_control_hdsc_hc32_data clock_control_hdsc_hc32_data = {};

DEVICE_DT_INST_DEFINE(0, clock_control_hdsc_hc32_init, NULL, &clock_control_hdsc_hc32_data,
		      &clock_control_hdsc_hc32_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_hdsc_hc32_api);
