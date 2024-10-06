/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hdsc_hc32_pinctrl

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#define CLOCK_CONTROLLER DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0))
#define GPIO_BASE DT_INST_REG_ADDR(0)

#define GPIO_REG_PA00_SEL 0x000
#define GPIO_REG_PAADS    0x10C
#define GPIO_REG_PAOUT    0x108
#define GPIO_REG_PADIR    0x100
#define GPIO_REG_PADR     0x11C
#define GPIO_REG_PAPU     0x120
#define GPIO_REG_PAPD     0x124
#define GPIO_REG_PAOD     0x12C

#define PXYY_SEL GENMASK(2, 0)

#define PORT_REG(pin, reg) (HC32_PINMUX_PORT((pin)) * 0x40 + (reg))

static void pinctrl_configure_pin(pinctrl_soc_pin_t pin)
{
	uint32_t value;
	mem_addr_t reg;

	/* Analog vs digital. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PAADS));
	/* Digital. */
	value &= ~BIT(HC32_PINMUX_PIN(pin));
	if (pin & BIT(HC32_ANALOG_MODE_POS)) {
		value |= BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PAADS));

	/* Output value. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PAOUT));
	/* Low. */
	value &= ~BIT(HC32_PINMUX_PIN(pin));
	if (pin & BIT(HC32_OUTPUT_HIGH_POS)) {
		value |= BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PAOUT));

	/* Input vs output. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PADIR));
	/* Input. */
	value |= BIT(HC32_PINMUX_PIN(pin));
	if (pin & BIT(HC32_OUTPUT_ENABLE_POS)) {
		value &= ~BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PADIR));

	/* Drive strength. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PADR));
	/* High drive. */
	value &= ~BIT(HC32_PINMUX_PIN(pin));
	if (pin & BIT(HC32_DRIVE_STRENGTH_POS)) {
		/* Low drive. */
		value |= BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PADR));

	/* Pull-down. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PAPD));
	/* Disabled. */
	value &= ~BIT(HC32_PINMUX_PIN(pin));
	if (!(pin & BIT(HC32_BIAS_DISABLE_POS)) && pin & BIT(HC32_BIAS_PULL_DOWN_POS)) {
		value |= BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PAPD));

	/* Pull-up. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PAPU));
	/* Disabled. */
	value &= ~BIT(HC32_PINMUX_PIN(pin));
	if (!(pin & BIT(HC32_BIAS_DISABLE_POS)) && pin & BIT(HC32_BIAS_PULL_UP_POS)) {
		value |= BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PAPU));

	/* Push-pull vs open-drain. */
	value = sys_read32(GPIO_BASE + PORT_REG(pin, GPIO_REG_PAOD));
	/* Push-pull. */
	value &= ~BIT(HC32_PINMUX_PIN(pin));
	if (pin & BIT(HC32_DRIVE_OPEN_DRAIN_POS)) {
		value |= BIT(HC32_PINMUX_PIN(pin));
	}
	sys_write32(value, GPIO_BASE + PORT_REG(pin, GPIO_REG_PAOD));

	/* Select function. */
	reg = GPIO_BASE + PORT_REG(pin, GPIO_REG_PA00_SEL) + (HC32_PINMUX_PIN(pin) << 2);
	value = sys_read32(reg);
	value &= ~PXYY_SEL;
	value |= FIELD_PREP(PXYY_SEL, HC32_PINMUX_AF(pin));
	sys_write32(value, reg);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);
	int ret;
	uint16_t clock_id = DT_INST_CLOCKS_CELL(0, clk_id);

	ret = clock_control_on(CLOCK_CONTROLLER, &clock_id);
	if (ret) {
		return ret;
	}

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins[i]);
	}

	return 0;
}
