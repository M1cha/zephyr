/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hdsc_hc32_gpio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/pinctrl/hc32-pinctrl.h>
#include <zephyr/dt-bindings/gpio/hc32-gpio.h>

#define REG_PA00_SEL  0x000
#define REG_PADIR     0x100
#define REG_PAIN      0x104
#define REG_PAOUT     0x108
#define REG_PAADS     0x10C
#define REG_PABSET    0x110
#define REG_PABCLR    0x114
#define REG_PABSETCLR 0x118
#define REG_PADR      0x11C
#define REG_PAPU      0x120
#define REG_PAPD      0x124
#define REG_PAOD      0x12C
#define REG_PAHIE     0x130
#define REG_PALIE     0x134
#define REG_PARIE     0x138
#define REG_PAFIE     0x13C
#define REG_PA_STAT   0x200
#define REG_PA_ICLR   0x210

#define PXYY_SEL GENMASK(2, 0)

#define PXBSETCLR_PXBCLR GENMASK(15, 0)
#define PXBSETCLR_PXBSET GENMASK(31, 16)

#define PIN_MASK              0xF
#define PORT_REG(config, reg) ((reg) + (config)->reg_offset)

struct gpio_hc32_config {
	struct gpio_driver_config common;
	mem_addr_t base;
	uint8_t reg_offset;
	const struct device *clock_dev;
	uint16_t clock_id;
};

struct gpio_hc32_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

static void gpio_hc32_isr(const struct device *port)
{
	const struct gpio_hc32_config *config = port->config;
	struct gpio_hc32_data *data = port->data;

	uint32_t status = sys_read32(config->base + PORT_REG(config, REG_PA_STAT)) & PIN_MASK;

	uint32_t value = sys_read32(config->base + PORT_REG(config, REG_PA_ICLR));
	value &= ~status;
	sys_write32(status, config->base + PORT_REG(config, REG_PA_ICLR));

	gpio_fire_callbacks(&data->callbacks, port, status);
}

static inline int gpio_hc32_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_hc32_config *config = port->config;
	int ret = 0;

	/* Simultaneous input & output mode not supported */
	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	uint32_t key = irq_lock();
	uint32_t dir = sys_read32(config->base + PORT_REG(config, REG_PADIR));
	uint32_t ads = sys_read32(config->base + PORT_REG(config, REG_PAADS));
	uint32_t od = sys_read32(config->base + PORT_REG(config, REG_PAOD));
	uint32_t out = sys_read32(config->base + PORT_REG(config, REG_PAOUT));
	uint32_t pu = sys_read32(config->base + PORT_REG(config, REG_PAPU));
	uint32_t pd = sys_read32(config->base + PORT_REG(config, REG_PAPD));
	uint32_t drive = sys_read32(config->base + PORT_REG(config, REG_PADR));

	if (flags & GPIO_OUTPUT) {
		ads &= ~BIT(pin);
		dir &= ~BIT(pin);

		if ((flags & GPIO_SINGLE_ENDED) != 0U) {
			if ((flags & GPIO_LINE_OPEN_DRAIN) != 0U) {
				od |= BIT(pin);
			} else {
				ret = -ENOTSUP;
				goto out_unlock;
			}
		} else {
			od &= ~BIT(pin);
		}

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			out |= BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			out &= ~BIT(pin);
		}
	} else if (flags & GPIO_INPUT) {
		ads &= ~BIT(pin);
		dir |= BIT(pin);
	} else {
		ads |= BIT(pin);
		dir |= BIT(pin);
	}

	if (flags & GPIO_PULL_UP) {
		pu |= BIT(pin);
	} else {
		pu &= ~BIT(pin);
	}

	if (flags & GPIO_PULL_DOWN) {
		pd |= BIT(pin);
	} else {
		pd &= ~BIT(pin);
	}

	if (flags & HC32_GPIO_DRIVE_LOW) {
		drive |= BIT(pin);
	} else {
		drive &= ~BIT(pin);
	}

	sys_write32(ads, config->base + PORT_REG(config, REG_PAADS));

	/* Switch to GPIO function. */
	mem_addr_t reg = config->base + PORT_REG(config, REG_PA00_SEL) + (LSB_GET(pin) << 2);
	uint32_t value = sys_read32(reg);
	value &= ~PXYY_SEL;
	value |= FIELD_PREP(PXYY_SEL, HC32_AF0);
	sys_write32(value, reg);

	sys_write32(out, config->base + PORT_REG(config, REG_PAOUT));
	sys_write32(dir, config->base + PORT_REG(config, REG_PADIR));
	sys_write32(drive, config->base + PORT_REG(config, REG_PADR));
	sys_write32(pu, config->base + PORT_REG(config, REG_PAPU));
	sys_write32(pd, config->base + PORT_REG(config, REG_PAPD));
	sys_write32(od, config->base + PORT_REG(config, REG_PAOD));

out_unlock:
	irq_unlock(key);
	return ret;
}

static int gpio_hc32_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_hc32_config *config = port->config;

	*value = sys_read32(config->base + PORT_REG(config, REG_PAIN)) & PIN_MASK;

	return 0;
}

static int gpio_hc32_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_hc32_config *config = port->config;
	const uint32_t set_mask = value & mask;
	const uint32_t clear_mask = (~set_mask) & mask;

	uint32_t reg =
		FIELD_PREP(PXBSETCLR_PXBCLR, clear_mask) | FIELD_PREP(PXBSETCLR_PXBSET, set_mask);
	sys_write32(reg, config->base + PORT_REG(config, REG_PABSETCLR));

	return 0;
}

static int gpio_hc32_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_hc32_config *config = port->config;

	sys_write32(pins, config->base + PORT_REG(config, REG_PABSET));

	return 0;
}

static int gpio_hc32_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_hc32_config *config = port->config;

	sys_write32(pins, config->base + PORT_REG(config, REG_PABCLR));

	return 0;
}

static int gpio_hc32_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_hc32_config *config = port->config;

	const uint32_t value =
		(sys_read32(config->base + PORT_REG(config, REG_PAOUT)) & PIN_MASK) ^ pins;
	const uint32_t set_mask = value & pins;
	const uint32_t clear_mask = (~value) & pins;

	uint32_t reg =
		FIELD_PREP(PXBSETCLR_PXBCLR, clear_mask) | FIELD_PREP(PXBSETCLR_PXBSET, set_mask);
	sys_write32(reg, config->base + PORT_REG(config, REG_PABSETCLR));

	return 0;
}

static int gpio_hc32_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_hc32_config *config = port->config;
	int ret = 0;

	uint32_t key = irq_lock();
	uint32_t hie = sys_read32(config->base + PORT_REG(config, REG_PAHIE));
	uint32_t lie = sys_read32(config->base + PORT_REG(config, REG_PALIE));
	uint32_t rie = sys_read32(config->base + PORT_REG(config, REG_PARIE));
	uint32_t fie = sys_read32(config->base + PORT_REG(config, REG_PAFIE));

	if (mode == GPIO_INT_MODE_DISABLED) {
		hie &= ~BIT(pin);
		lie &= ~BIT(pin);
		rie &= ~BIT(pin);
		fie &= ~BIT(pin);
	} else if (mode == GPIO_INT_MODE_EDGE) {
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			fie |= BIT(pin);
			break;
		case GPIO_INT_TRIG_HIGH:
			rie |= BIT(pin);
			break;
		case GPIO_INT_TRIG_BOTH:
			rie |= BIT(pin);
			fie |= BIT(pin);
			break;
		default:
			ret = -ENOTSUP;
			goto out_unlock;
		}
	} else if (mode == GPIO_INT_MODE_LEVEL) {
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			lie |= BIT(pin);
			break;
		case GPIO_INT_TRIG_HIGH:
			hie |= BIT(pin);
			break;
		case GPIO_INT_TRIG_BOTH:
			lie |= BIT(pin);
			hie |= BIT(pin);
			break;
		default:
			ret = -ENOTSUP;
			goto out_unlock;
		}
	} else {
		ret = -ENOTSUP;
		goto out_unlock;
	}

	sys_write32(hie, config->base + PORT_REG(config, REG_PAHIE));
	sys_write32(lie, config->base + PORT_REG(config, REG_PALIE));
	sys_write32(rie, config->base + PORT_REG(config, REG_PARIE));
	sys_write32(fie, config->base + PORT_REG(config, REG_PAFIE));

out_unlock:
	irq_unlock(key);
	return ret;
}

static int gpio_hc32_manage_callback(const struct device *port, struct gpio_callback *callback,
				     bool set)
{
	struct gpio_hc32_data *data = port->data;
	return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_hc32_api = {
	.pin_configure = gpio_hc32_configure,
	.port_get_raw = gpio_hc32_port_get_raw,
	.port_set_masked_raw = gpio_hc32_port_set_masked_raw,
	.port_set_bits_raw = gpio_hc32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_hc32_port_clear_bits_raw,
	.port_toggle_bits = gpio_hc32_port_toggle_bits,
	.pin_interrupt_configure = gpio_hc32_pin_interrupt_configure,
	.manage_callback = gpio_hc32_manage_callback,
};

#define GPIO_HC32_DEFINE(inst)                                                                     \
	static const struct gpio_hc32_config gpio_hc32_config_##inst = {                           \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.base = DT_REG_ADDR(DT_INST_PARENT(inst)),                                         \
		.reg_offset = DT_INST_REG_ADDR(inst),                                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_id = DT_INST_CLOCKS_CELL(inst, clk_id),                                     \
	};                                                                                         \
                                                                                                   \
	static struct gpio_hc32_data gpio_hc32_data_##inst;                                        \
                                                                                                   \
	static int gpio_hc32_init_##inst(const struct device *port)                                \
	{                                                                                          \
		const struct gpio_hc32_config *config = port->config;                              \
                                                                                                   \
		uint16_t clock_id = config->clock_id;                                              \
		int ret = clock_control_on(config->clock_dev, &clock_id);                          \
		if (ret) {                                                                         \
			return ret;                                                                \
		}                                                                                  \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), gpio_hc32_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_hc32_init_##inst, NULL, &gpio_hc32_data_##inst,           \
			      &gpio_hc32_config_##inst, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,   \
			      &gpio_hc32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_HC32_DEFINE)
