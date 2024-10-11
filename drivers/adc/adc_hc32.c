/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hdsc_hc32_adc

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_hc32, CONFIG_ADC_LOG_LEVEL);

#define REG_BGR_CR   0x000
#define REG_CR0      0x004
#define REG_CR1      0x008
#define REG_RESULT   0x0A0
#define REG_IFR      0x0B0
#define REG_ICR      0x0B4
#define REG_SGLSTART 0x0C0

#define BGR_CR_BGR_EN BIT(0)
#define BGR_CR_TS_EN  BIT(1)

#define CR0_EN        BIT(0)
#define CR0_CKDIV     GENMASK(3, 2)
#define CR0_SGLMUX    GENMASK(8, 4)
#define CR0_REF       GENMASK(10, 9)
#define CR0_BUF       BIT(11)
#define CR0_SAM       GENMASK(13, 12)
#define CR0_IN_REF_EN BIT(14)
#define CR0_IE        BIT(15)

#define CR1_ALIGN     BIT(2)
#define CR1_TH_CH     GENMASK(7, 3)
#define CR1_DMA_SQR   BIT(8)
#define CR1_DMA_JQR   BIT(9)
#define CR1_MODE      BIT(10)
#define CR1_R_ACC_EN  BIT(11)
#define CR1_LT_CMP    BIT(12)
#define CR1_HT_CMP    BIT(13)
#define CR1_REG_CMP   BIT(14)
#define CR1_R_REG_CLR BIT(15)

#define RESULT_RESULT GENMASK(11, 0)

#define IFR_SGLIF BIT(0)
#define IFR_LTIF  BIT(1)
#define IFR_HTIF  BIT(2)
#define IFR_REGIF BIT(3)
#define IFR_SQRIF BIT(4)
#define IFR_JQRIF BIT(5)

#define ICR_SGLIC BIT(0)
#define ICR_LTIC  BIT(1)
#define ICR_HTIC  BIT(2)
#define ICR_REGIC BIT(3)
#define ICR_SQRIC BIT(4)
#define ICR_JQRIC BIT(5)

#define SGLSTART_START BIT(0)

struct adc_hc32_channel {
	uint8_t buf: 1;
	uint8_t ref: 2;
	uint8_t sam: 2;
	uint8_t in_ref_en: 1;
};

struct adc_hc32_config {
	uint32_t base;
	const struct device *clock_dev;
	uint16_t clock_id;
	struct adc_hc32_channel *channels;
	size_t num_channels;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(void);

	bool temp_sensor_enabled;
	uint8_t clock_div;
};

struct adc_hc32_data {
	struct adc_context ctx;
	const struct device *dev;
	uint32_t channels;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
};

static inline void adc_hc32_set_interrupt_enabled(const struct device *dev, bool enable)
{
	const struct adc_hc32_config *config = dev->config;
	uint32_t value = sys_read32(config->base + REG_CR0);
	if (enable) {
		value |= CR0_IE;
	} else {
		value &= ~CR0_IE;
	}
	sys_write32(value, config->base + REG_CR0);
}

static inline void clear_icr(const struct device *dev, uint32_t mask)
{
	const struct adc_hc32_config *config = dev->config;

	uint32_t icr = sys_read32(config->base + REG_ICR);
	icr &= ~mask;
	sys_write32(icr, config->base + REG_ICR);
}

static void start_next_sample(const struct device *dev)
{
	struct adc_hc32_data *data = dev->data;
	const struct adc_hc32_config *config = dev->config;

	if (!data->channels) {
		adc_context_on_sampling_done(&data->ctx, dev);
		return;
	}

	unsigned int channel_id = find_lsb_set(data->channels) - 1;
	WRITE_BIT(data->channels, channel_id, 0);
	struct adc_hc32_channel *channel = &config->channels[channel_id];

	uint32_t cr0 = sys_read32(config->base + REG_CR0);
	uint32_t cr1 = sys_read32(config->base + REG_CR1);
	cr1 &= ~CR1_MODE;
	cr1 &= ~CR1_LT_CMP;
	cr1 &= ~CR1_HT_CMP;
	cr1 &= ~CR1_REG_CMP;

	if (channel->buf) {
		cr0 |= CR0_BUF;
	} else {
		cr0 &= ~CR0_BUF;
	}

	cr0 &= ~CR0_REF;
	cr0 |= FIELD_PREP(CR0_REF, channel->ref);

	cr0 &= ~CR0_IN_REF_EN;
	if (channel->in_ref_en) {
		cr0 |= CR0_IN_REF_EN;
	}

	cr0 &= ~CR0_SAM;
	cr0 |= FIELD_PREP(CR0_SAM, channel->sam);

	cr0 &= ~CR0_SGLMUX;
	cr0 |= FIELD_PREP(CR0_SGLMUX, channel_id);

	sys_write32(cr0, config->base + REG_CR0);
	sys_write32(cr1, config->base + REG_CR1);

	uint32_t sglstart = sys_read32(config->base + REG_SGLSTART);
	sglstart |= SGLSTART_START;
	sys_write32(sglstart, config->base + REG_SGLSTART);

	adc_hc32_set_interrupt_enabled(dev, true);
}

static void adc_hc32_irq_handler(const struct device *dev)
{
	struct adc_hc32_data *data = dev->data;
	const struct adc_hc32_config *config = dev->config;

	uint32_t ifr = sys_read32(config->base + REG_IFR);

	if (ifr & IFR_SGLIF) {
		uint32_t value = sys_read32(config->base + REG_RESULT);
		*data->buffer++ = FIELD_GET(RESULT_RESULT, value);

		adc_hc32_set_interrupt_enabled(dev, false);
		clear_icr(dev, ICR_SGLIC);

		start_next_sample(dev);
	}

	if (ifr & IFR_LTIF) {
		clear_icr(dev, ICR_LTIC);
	}
	if (ifr & IFR_HTIF) {
		clear_icr(dev, ICR_HTIC);
	}
	if (ifr & IFR_REGIF) {
		clear_icr(dev, ICR_REGIC);
	}
	if (ifr & IFR_SQRIF) {
		clear_icr(dev, ICR_SQRIC);
	}
	if (ifr & IFR_JQRIF) {
		clear_icr(dev, ICR_JQRIC);
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_hc32_data *data = CONTAINER_OF(ctx, struct adc_hc32_data, ctx);
	const struct device *dev = data->dev;

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	start_next_sample(dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_hc32_data *data = CONTAINER_OF(ctx, struct adc_hc32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int adc_hc32_channel_setup(const struct device *dev, const struct adc_channel_cfg *chan_cfg)
{
	const struct adc_hc32_config *config = dev->config;

	if (chan_cfg->channel_id >= config->num_channels) {
		LOG_ERR("Invalid channel (%u)", chan_cfg->channel_id);
		return -EINVAL;
	}
	struct adc_hc32_channel *channel = &config->channels[chan_cfg->channel_id];

	*channel = (struct adc_hc32_channel){};

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported gain: %u", chan_cfg->gain);
		return -ENOTSUP;
	}

	if (chan_cfg->use_buffer_amplifier) {
		channel->buf = 1;
	} else {
		channel->buf = 0;
	}

	switch (chan_cfg->reference) {
	case ADC_REF_VDD_1:
		channel->ref = 3;
		break;
	case ADC_REF_EXTERNAL0:
		channel->ref = 2;
		break;
	case ADC_REF_INTERNAL0:
		/* 1.5V */
		channel->ref = 0;
		channel->in_ref_en = 1;
		break;
	case ADC_REF_INTERNAL1:
		/* 2.5V */
		channel->ref = 1;
		channel->in_ref_en = 1;
		break;
	default:
		LOG_ERR("Unsupported reference: %u", chan_cfg->reference);
		return -ENOTSUP;
	}

	if (chan_cfg->acquisition_time == ADC_ACQ_TIME_DEFAULT) {
		channel->sam = 0;
	} else if (ADC_ACQ_TIME_UNIT(chan_cfg->acquisition_time) == ADC_ACQ_TIME_TICKS) {
		switch (ADC_ACQ_TIME_VALUE(chan_cfg->acquisition_time)) {
		case 4:
			channel->sam = 0;
			break;
		case 6:
			channel->sam = 1;
			break;
		case 8:
			channel->sam = 2;
			break;
		case 12:
			channel->sam = 3;
			break;
		default:
			LOG_ERR("Unsupported acquisition time: %lu ticks",
				ADC_ACQ_TIME_VALUE(chan_cfg->acquisition_time));
			return -EINVAL;
		}
	} else {
		LOG_ERR("Unsupported acquisition time unit: %lu",
			ADC_ACQ_TIME_UNIT(chan_cfg->acquisition_time));
		return -EINVAL;
	}

	if (chan_cfg->differential) {
		LOG_ERR("Differential mode is not supported");
		return -EINVAL;
	}

	return 0;
}

static int check_buffer_size(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_hc32_config *config = dev->config;
	uint8_t channels = 0;
	size_t needed;
	uint32_t mask;

	for (mask = BIT(config->num_channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(uint16_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int adc_hc32_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_hc32_data *data = dev->data;
	const struct adc_hc32_config *config = dev->config;
	int ret;

	if (find_msb_set(sequence->channels) > config->num_channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x", sequence->channels);
		return -ENOTSUP;
	}
	if (sequence->channels == 0) {
		LOG_ERR("No channels selected");
		return -ENOTSUP;
	}

	ret = check_buffer_size(dev, sequence);
	if (ret) {
		LOG_ERR("buffer size too small");
		return ret;
	}

	if (sequence->resolution != 12) {
		LOG_ERR("Unsupported resolution: %u", sequence->resolution);
		return -EINVAL;
	}

	if (sequence->oversampling) {
		LOG_ERR("oversampling is not supported");
		return -EINVAL;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_hc32_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_hc32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_hc32_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_hc32_read_async(const struct device *dev, const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_hc32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_hc32_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static struct adc_driver_api adc_hc32_driver_api = {
	.channel_setup = adc_hc32_channel_setup,
	.read = adc_hc32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_hc32_read_async,
#endif /* CONFIG_ADC_ASYNC */
};

static int adc_hc32_init(const struct device *dev)
{
	struct adc_hc32_data *data = dev->data;
	const struct adc_hc32_config *config = dev->config;
	int ret;
	uint32_t value;

	data->dev = dev;

	uint16_t clock_id = config->clock_id;
	ret = clock_control_on(config->clock_dev, &clock_id);
	if (ret) {
		LOG_ERR("Failed to enable module clock: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to enable apply default pinctrl state: %d", ret);
		return ret;
	}

	value = sys_read32(config->base + REG_CR0);
	value |= CR0_EN;
	value &= ~CR0_CKDIV;
	value |= FIELD_PREP(CR0_CKDIV, config->clock_div);
	sys_write32(value, config->base + REG_CR0);

	value = sys_read32(config->base + REG_BGR_CR);
	value |= BGR_CR_BGR_EN;
	if (config->temp_sensor_enabled) {
		value |= BGR_CR_TS_EN;
	} else {
		value &= ~BGR_CR_TS_EN;
	}
	sys_write32(value, config->base + REG_BGR_CR);
	k_busy_wait(20);

	config->irq_config_func();
	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define ADC_HC32_IRQ_HANDLER(inst)                                                                 \
	static void adc_hc32_irq_config_func_##inst(void)                                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), adc_hc32_irq_handler, \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define ADC_HC32_INIT(inst)                                                                        \
	ADC_HC32_IRQ_HANDLER(inst);                                                                \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static struct adc_hc32_data adc_hc32_data_##inst = {                                       \
		ADC_CONTEXT_INIT_TIMER(adc_hc32_data_##inst, ctx),                                 \
		ADC_CONTEXT_INIT_LOCK(adc_hc32_data_##inst, ctx),                                  \
		ADC_CONTEXT_INIT_SYNC(adc_hc32_data_##inst, ctx),                                  \
	};                                                                                         \
                                                                                                   \
	static struct adc_hc32_channel channels[DT_INST_PROP(inst, channels)];                     \
	const static struct adc_hc32_config adc_hc32_config_##inst = {                             \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_id = DT_INST_CLOCKS_CELL(inst, clk_id),                                     \
		.channels = channels,                                                              \
		.num_channels = DT_INST_PROP(inst, channels),                                      \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.irq_config_func = adc_hc32_irq_config_func_##inst,                                \
		.clock_div = DT_INST_ENUM_IDX_OR(inst, clock_div, 0),                              \
		.temp_sensor_enabled = DT_INST_PROP(inst, temp_sensor_enabled),                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &adc_hc32_init, NULL, &adc_hc32_data_##inst,                   \
			      &adc_hc32_config_##inst, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,      \
			      &adc_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_HC32_INIT)
