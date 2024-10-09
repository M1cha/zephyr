/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hdsc_hc32_lpuart

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/hdsc_hc32_clock.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>

#define REG_SBUF  0x00
#define REG_SCON  0x04
#define REG_SADDR 0x08
#define REG_SADEN 0x0C
#define REG_ISR   0x10
#define REG_ICR   0x14
#define REG_SCNT  0x18

#define SBUF_DATA  GENMASK(7, 0)
#define SBUF_DATA8 BIT(8)

#define SCON_RCIE    BIT(0)
#define SCON_TCIE    BIT(1)
#define SCON_B8CONT  GENMASK(3, 2)
#define SCON_REN     BIT(4)
#define SCON_ADRDET  BIT(5)
#define SCON_SM      GENMASK(7, 6)
#define SCON_TXEIE   BIT(8)
#define SCON_OVER    GENMASK(10, 9)
#define SCON_SCLKSEL GENMASK(12, 11)
#define SCON_PEIE    BIT(13)
#define SCON_STOPBIT GENMASK(15, 14)
#define SCON_DMARXEN BIT(16)
#define SCON_DMATXEN BIT(17)
#define SCON_RTSEN   BIT(18)
#define SCON_CTSEN   BIT(19)
#define SCON_CTSIE   BIT(20)
#define SCON_FEIE    BIT(21)

#define SADDR_SADDR GENMASK(7, 0)

#define SADEN_SADEN GENMASK(7, 0)

#define ISR_RC    BIT(0)
#define ISR_TC    BIT(1)
#define ISR_FE    BIT(2)
#define ISR_TXE   BIT(3)
#define ISR_PE    BIT(4)
#define ISR_CTSIF BIT(5)
#define ISR_CTS   BIT(6)

#define ICR_RCCF    BIT(0)
#define ICR_TCCF    BIT(1)
#define ICR_FECF    BIT(2)
#define ICR_PECF    BIT(4)
#define ICR_CTSIFCF BIT(5)

#define SCNT_SCNT GENMASK(15, 0)

#define ABS_DIFF(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))

struct best_scnt {
	uint16_t scnt;
	uint32_t baud_diff;
	uint8_t over;
};

struct lpuart_hc32_config {
	mem_addr_t base;
	uint32_t baud_rate;
	bool hw_flow_control;
	uint8_t parity;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_config_func_t irq_config_func;
#endif
	const struct device *module_clock_dev;
	uint16_t module_clock_id;

	const struct device *tx_clock_dev;
	uint16_t tx_clock_id;

	const struct pinctrl_dev_config *pincfg;
};

struct lpuart_hc32_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

static void clear_isr(const struct device *dev, uint32_t mask)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t icr = sys_read32(config->base + REG_ICR);
	icr &= ~mask;
	sys_write32(icr, config->base + REG_ICR);
}

static int lpuart_hc32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct lpuart_hc32_config *config = dev->config;
	int ret = -1;

	uint32_t key = irq_lock();
	uint32_t isr = sys_read32(config->base + REG_ISR);
	if (FIELD_GET(ISR_RC, isr)) {
		uint32_t sbuf = sys_read32(config->base + REG_SBUF);
		*c = FIELD_GET(SBUF_DATA, sbuf);
		ret = 0;

		clear_isr(dev, ICR_RCCF);
	}

	irq_unlock(key);
	return ret;
}

static void lpuart_hc32_poll_out(const struct device *dev, unsigned char c)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t key = irq_lock();

	while (!(sys_read32(config->base + REG_ISR) & ISR_TXE))
		;

	uint32_t sbuf = sys_read32(config->base + REG_SBUF);
	sbuf &= ~SBUF_DATA;
	sbuf |= FIELD_PREP(SBUF_DATA, c);
	sys_write32(sbuf, config->base + REG_SBUF);

	irq_unlock(key);
}

static int lpuart_hc32_err_check(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;
	int ret = 0;

	uint32_t isr = sys_read32(config->base + REG_ISR);

	if (isr & ISR_PE) {
		clear_isr(dev, ICR_PECF);
		ret |= UART_ERROR_PARITY;
	}

	if (isr & ISR_FE) {
		clear_isr(dev, ICR_FECF);
		ret |= UART_ERROR_FRAMING;
	}

	return ret;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int lpuart_hc32_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct lpuart_hc32_config *config = dev->config;

	if (size < 1) {
		return 0;
	}

	uint32_t sbuf = sys_read32(config->base + REG_SBUF);
	sbuf &= ~SBUF_DATA;
	sbuf |= FIELD_PREP(SBUF_DATA, tx_data[0]);
	sys_write32(sbuf, config->base + REG_SBUF);

	return 1;
}

static int lpuart_hc32_irq_rx_ready(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t isr = sys_read32(config->base + REG_ISR);
	return isr & ISR_RC;
}

static int lpuart_hc32_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct lpuart_hc32_config *config = dev->config;

	if (size < 1) {
		return 0;
	}
	if (!lpuart_hc32_irq_rx_ready(dev)) {
		return 0;
	}

	uint32_t sbuf = sys_read32(config->base + REG_SBUF);
	rx_data[0] = FIELD_GET(SBUF_DATA, sbuf);
	clear_isr(dev, ICR_RCCF);

	return 1;
}

static void lpuart_hc32_irq_tx_enable(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t scon = sys_read32(config->base + REG_SCON);
	scon |= SCON_TXEIE;
	sys_write32(scon, config->base + REG_SCON);
}

static void lpuart_hc32_irq_tx_disable(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t scon = sys_read32(config->base + REG_SCON);
	scon &= ~SCON_TXEIE;
	sys_write32(scon, config->base + REG_SCON);
}

static int lpuart_hc32_irq_tx_ready(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t isr = sys_read32(config->base + REG_ISR);
	return isr & ISR_TXE;
}

static int lpuart_hc32_irq_tx_complete(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t isr = sys_read32(config->base + REG_ISR);
	return isr & ISR_TC;
}

static void lpuart_hc32_irq_rx_enable(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t scon = sys_read32(config->base + REG_SCON);
	scon |= SCON_RCIE;
	sys_write32(scon, config->base + REG_SCON);
}

static void lpuart_hc32_irq_rx_disable(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t scon = sys_read32(config->base + REG_SCON);
	scon &= ~SCON_RCIE;
	sys_write32(scon, config->base + REG_SCON);
}

static void lpuart_hc32_irq_err_enable(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t scon = sys_read32(config->base + REG_SCON);
	scon |= SCON_FEIE;
	scon |= SCON_PEIE;
	sys_write32(scon, config->base + REG_SCON);
}

static void lpuart_hc32_irq_err_disable(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t scon = sys_read32(config->base + REG_SCON);
	scon &= ~SCON_FEIE;
	scon &= ~SCON_PEIE;
	sys_write32(scon, config->base + REG_SCON);
}

static int lpuart_hc32_irq_is_pending(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;

	uint32_t isr = sys_read32(config->base + REG_ISR);
	return !!isr;
}

static int lpuart_hc32_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void lpuart_hc32_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					 void *cb_data)
{
	struct lpuart_hc32_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void lpuart_hc32_irq_handler(const struct device *dev)
{
	struct lpuart_hc32_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}

	lpuart_hc32_err_check(dev);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api lpuart_hc32_api = {
	.poll_in = lpuart_hc32_poll_in,
	.poll_out = lpuart_hc32_poll_out,
	.err_check = lpuart_hc32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = lpuart_hc32_fifo_fill,
	.fifo_read = lpuart_hc32_fifo_read,
	.irq_tx_enable = lpuart_hc32_irq_tx_enable,
	.irq_tx_disable = lpuart_hc32_irq_tx_disable,
	.irq_tx_ready = lpuart_hc32_irq_tx_ready,
	.irq_tx_complete = lpuart_hc32_irq_tx_complete,
	.irq_rx_enable = lpuart_hc32_irq_rx_enable,
	.irq_rx_disable = lpuart_hc32_irq_rx_disable,
	.irq_rx_ready = lpuart_hc32_irq_rx_ready,
	.irq_err_enable = lpuart_hc32_irq_err_enable,
	.irq_err_disable = lpuart_hc32_irq_err_disable,
	.irq_is_pending = lpuart_hc32_irq_is_pending,
	.irq_update = lpuart_hc32_irq_update,
	.irq_callback_set = lpuart_hc32_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static uint16_t calc_scnt(uint32_t sclk_rate, uint32_t baud_rate, uint8_t over)
{
	return sclk_rate / (baud_rate * over);
}

static uint32_t calc_baud(uint32_t sclk_rate, uint16_t scnt, uint8_t over)
{
	return sclk_rate / (scnt * over);
}

static void select_scnt(struct best_scnt *best_scnt, uint32_t sclk_rate, uint32_t baud_rate,
			uint8_t over)
{
	uint16_t scnt = calc_scnt(sclk_rate, baud_rate, over);
	uint32_t baud_scnt = calc_baud(sclk_rate, scnt, over);
	uint32_t baud_scnt_plus1 = calc_baud(sclk_rate, scnt + 1, over);

	uint32_t tmp_baud_diff = ABS_DIFF(baud_rate, baud_scnt);
	if (tmp_baud_diff < best_scnt->baud_diff) {
		best_scnt->baud_diff = tmp_baud_diff;
		best_scnt->scnt = scnt;
		best_scnt->over = over;
	}

	tmp_baud_diff = ABS_DIFF(baud_rate, baud_scnt_plus1);
	if (tmp_baud_diff < best_scnt->baud_diff) {
		best_scnt->baud_diff = tmp_baud_diff;
		best_scnt->scnt = scnt + 1;
		best_scnt->over = over;
	}
}

static int lpuart_hc32_init(const struct device *dev)
{
	const struct lpuart_hc32_config *config = dev->config;
	int ret;
	uint16_t clock_id;

	if (!device_is_ready(config->module_clock_dev)) {
		return -ENODEV;
	}

	if (!device_is_ready(config->tx_clock_dev)) {
		return -ENODEV;
	}

	uint8_t sclksel;
	switch (config->tx_clock_id) {
	case HDSC_HC32_CLKID_PERI:
		sclksel = 0;
		break;
	case HDSC_HC32_CLKID_XTL:
		sclksel = 2;
		break;
	case HDSC_HC32_CLKID_RCL:
		sclksel = 3;
		break;
	default:
		return -ENOTSUP;
	}

	clock_id = config->tx_clock_id;
	uint32_t sclk_rate;
	ret = clock_control_get_rate(config->tx_clock_dev, &clock_id, &sclk_rate);
	if (ret) {
		return ret;
	}

	struct best_scnt best_scnt = {
		.baud_diff = UINT32_MAX,
		.scnt = 0,
	};

	select_scnt(&best_scnt, sclk_rate, config->baud_rate, 4);
	select_scnt(&best_scnt, sclk_rate, config->baud_rate, 8);
	select_scnt(&best_scnt, sclk_rate, config->baud_rate, 16);

	uint8_t over_reg;
	switch (best_scnt.over) {
	case 4:
		over_reg = 2;
		break;
	case 8:
		over_reg = 1;
		break;
	case 16:
		over_reg = 0;
		break;
	default:
		__ASSERT_NO_MSG(0);
		return -EINVAL;
	}

	clock_id = config->module_clock_id;
	ret = clock_control_on(config->module_clock_dev, &clock_id);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	uint32_t scon = sys_read32(config->base + REG_SCON);

	scon &= ~SCON_SM;
	scon &= ~SCON_B8CONT;
	switch (config->parity) {
	case UART_CFG_PARITY_NONE:
		scon |= FIELD_PREP(SCON_SM, 1);
		break;
	case UART_CFG_PARITY_ODD:
		scon |= FIELD_PREP(SCON_SM, 3);
		scon |= FIELD_PREP(SCON_B8CONT, 2);
		break;
	case UART_CFG_PARITY_EVEN:
		scon |= FIELD_PREP(SCON_SM, 3);
		scon |= FIELD_PREP(SCON_B8CONT, 1);
		break;
	default:
		return -ENOTSUP;
	}

	scon &= ~SCON_STOPBIT;
	scon |= FIELD_PREP(SCON_STOPBIT, 0);

	if (config->hw_flow_control) {
		scon |= FIELD_PREP(SCON_CTSEN, 1);
		scon |= FIELD_PREP(SCON_RTSEN, 1);
	} else {
		scon &= ~SCON_CTSEN;
		scon &= ~SCON_RTSEN;
	}

	scon |= FIELD_PREP(SCON_REN, 1);

	scon &= ~SCON_SCLKSEL;
	scon |= FIELD_PREP(SCON_SCLKSEL, sclksel);

	scon &= ~SCON_OVER;
	scon |= FIELD_PREP(SCON_OVER, over_reg);

	sys_write32(scon, config->base + REG_SCON);

	uint32_t scnt = sys_read32(config->base + REG_SCNT);
	scnt &= ~SCNT_SCNT;
	scnt |= FIELD_PREP(SCNT_SCNT, best_scnt.scnt);
	sys_write32(scnt, config->base + REG_SCNT);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	config->irq_config_func(dev);
#endif

	return 0;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
#define LPUART_HC32_IRQ_HANDLER(inst)                                                              \
	static void lpuart_hc32_irq_config_func_##inst(const struct device *dev)                   \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                       \
			    lpuart_hc32_irq_handler, DEVICE_DT_INST_GET(inst), 0);                 \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}
#define LPUART_HC32_IRQ_HANDLER_FUNC(inst) .irq_config_func = lpuart_hc32_irq_config_func_##inst,
#else
#define LPUART_HC32_IRQ_HANDLER(inst)      /* Not used */
#define LPUART_HC32_IRQ_HANDLER_FUNC(inst) /* Not used */
#endif

#define LPUART_HC32_INIT(inst)                                                                     \
	LPUART_HC32_IRQ_HANDLER(inst);                                                             \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct lpuart_hc32_config lpuart_hc32_config_##inst = {                       \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.baud_rate = DT_INST_PROP(inst, current_speed),                                    \
		.hw_flow_control = DT_INST_PROP(inst, hw_flow_control),                            \
		.parity = DT_INST_ENUM_IDX_OR(inst, parity, UART_CFG_PARITY_NONE),                 \
		.module_clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, module)),      \
		.module_clock_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, module, clk_id),              \
		.tx_clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, tx)),              \
		.tx_clock_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, tx, clk_id),                      \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		LPUART_HC32_IRQ_HANDLER_FUNC(inst)};                                               \
                                                                                                   \
	static struct lpuart_hc32_data lpuart_hc32_data_##inst;                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &lpuart_hc32_init, NULL, &lpuart_hc32_data_##inst,             \
			      &lpuart_hc32_config_##inst, PRE_KERNEL_1,                            \
			      CONFIG_SERIAL_INIT_PRIORITY, &lpuart_hc32_api);

DT_INST_FOREACH_STATUS_OKAY(LPUART_HC32_INIT)
