// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Analog Devices, Inc.
 * Author: Radu Sabau <radu.sabau@analog.com>
 */

#include <linux/math.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
// #include <linux/spi/spi-engine.h>
#include <linux/util_macros.h>
#include <linux/units.h>
#include <linux/types.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>

#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define AD4692_NUM_REGULATORS			1
#define AD4692_MAX_ADC_MODE			4
#define AD4692_GP_MODE_NUM			4

#define AD4692_VREF_MIN				2400000
#define AD4692_VREF_MAX				5250000

#define AD4692_CNV_DUTY_CYCLE_NS		210
#define AD4692_CONV_PERIOD_NS			1000
#define AD4692_OSC_PERIOD_NS(x)			DIV_ROUND_CLOSEST_ULL(NANO, x)

#define AD4692_CONV_START			0x01
#define AD4692_CONV_STOP			0x00
#define AD4692_SEQ_ALL_CHANNELS_OFF		0x00
#define AD4692_STATE_RESET_ALL			0x01

#define AD4692_ADC_MODE_MASK			GENMASK(1, 0)
#define AD4692_CONV_START_MASK			BIT(0)
#define AD4692_STOP_STATE_MASK			BIT(0)
#define AD4692_MANUAL_MODE_MASK			BIT(2)
#define AD4692_REF_CTRL_MASK			GENMASK(4, 2)

/* Manual Mode Commands & Masks */
#define AD4692_COMMAND_MASK			GENMASK(7, 3)
#define AD4692_ADC_DATA_14_LSB_MASK		GENMASK(7, 2)
#define AD4692_ADC_DATA_18_LSB_MASK		GENMASK(7, 6)
#define AD4692_CHAN_MASK			GENMASK(3, 0)
#define AD4692_OV_ERR_MASK			BIT(4)

#define AD4692_NOOP				0x00
#define AD4692_EXIT_COMMAND			0x0A
#define AD4692_TEMPERATURE_SENSOR		0x0F
#define AD4692_ADC_CHAN(ch)			(0x10 + (ch))

#define AD4692_INTERFACE_CONFIG_A_REG		0x00

#define AD4692_STATUS_REG			0x14
#define AD4692_CLAMP_STATUS1_REG		0x1A
#define AD4692_CLAMP_STATUS2_REG		0x1B
#define AD4692_ADC_SETUP			0x20
#define AD4692_REFERENCE_CONTROL		0x21
#define AD4692_SEQUENCER_CONTROL		0x22
#define AD4692_INTERNAL_OSCILLATOR		0x23
#define AD4692_STD_SEQ_CONFIG			0x24
#define AD4692_SPARE_CONTROL			0x2A
#define AD4692_CHANNEL_CONFIG(n)		(0x30 + (n))
#define AD4692_IN_SELECT_AS(n)			(0x100 + (n))

#define AD4692_CONV_START_REG			0x180
#define AD4692_STATE_RESET_REG			0x181
#define AD4692_ADC_OPERATION_REG		0x182
#define AD4692_ACC_MASK1_REG			0x184
#define AD4692_ACC_MASK2_REG			0x185
#define AD4692_ACC_COUNT_LIMIT(n)		(0x186 + (n))
#define AD4692_GPIO_MODE1_REG			0x196
#define AD4692_GPIO_MODE2_REG			0x197
#define AD4692_GPIO_READ			0x1A0
#define AD4692_ACC_STATUS_FULL1_REG		0x1B0
#define AD4692_ACC_STATUS_FULL2_REG		0x1B1
#define AD4692_ACC_STATUS_OVERRUN1_REG		0x1B2
#define AD4692_ACC_STATUS_OVERRUN2_REG		0x1B3
#define AD4692_ACC_STATUS_SAT1_REG		0x1B4
#define AD4692_ACC_STATUS_SAT2_REG		0x1B5
#define AD4692_ACC_SAT_OVR_REG(n)		(0x1C0 + (n))
#define AD4692_IN_ACC_16B(n)			(0x200 + 2 * (n))
#define AD4692_IN_ACC_16B_STATUS(n)		(0x220 + 3 * (n))
#define AD4692_IN_ACC_24B(n)			(0x250 + 3 * (n))
#define AD4692_IN_ACC_24B_STATUS(n)		(0x280 + 4 * (n))

#define AD4692_ADDR_ASCENSION			BIT(5)

#define AD4692_ACC_MASK1_MASK			GENMASK(7, 0)
#define AD4692_ACC_MASK2_MASK			GENMASK(15, 8)

#define AD4692_GPIO_MODE_MASK(offset)		GENMASK(3 + 4 * (offset % 2), 0 + 4 * (offset % 2))
#define AD4692_GPIO_STATE(offset)		BIT(offset)
#define AD4692_GPIO_STATE_REG			0x30
#define AD4692_GPIO_STATE_EN_REG		0x60
#define AD4692_GPIO_STATE_EN(offset)		BIT(offset)
#define AD4692_GPIO_DIR_REG			0x70
#define AD4692_GPIO_DIR(offset)			BIT(offset)

#define AD4692_DATA_STOP_STATE			BIT(5)

#define AD4692_CHANNEL(index, real_bits, storage_bits)			\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ)	\
					   | BIT(IIO_CHAN_INFO_SCALE),	\
		.channel = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = real_bits,				\
			.storagebits = storage_bits,			\
		},							\
	}

#define AD4694_CHANNELS(real_bits, storage_bits)	\
	AD4692_CHANNEL(0, real_bits, storage_bits),	\
	AD4692_CHANNEL(1, real_bits, storage_bits),	\
	AD4692_CHANNEL(2, real_bits, storage_bits),	\
	AD4692_CHANNEL(3, real_bits, storage_bits),	\
	AD4692_CHANNEL(4, real_bits, storage_bits),	\
	AD4692_CHANNEL(5, real_bits, storage_bits),	\
	AD4692_CHANNEL(6, real_bits, storage_bits),	\
	AD4692_CHANNEL(7, real_bits, storage_bits)

#define AD4692_CHANNELS(real_bits, storage_bits)	\
	AD4694_CHANNELS(real_bits, storage_bits),	\
	AD4692_CHANNEL(8, real_bits, storage_bits),	\
	AD4692_CHANNEL(9, real_bits, storage_bits),	\
	AD4692_CHANNEL(10, real_bits, storage_bits),	\
	AD4692_CHANNEL(11, real_bits, storage_bits),	\
	AD4692_CHANNEL(12, real_bits, storage_bits),	\
	AD4692_CHANNEL(13, real_bits, storage_bits),	\
	AD4692_CHANNEL(14, real_bits, storage_bits),	\
	AD4692_CHANNEL(15, real_bits, storage_bits)

enum ad4692_ids {
	ID_AD4692,
	ID_AD4691,
	ID_AD4694,
	ID_AD4693
};

enum ad4692_adc_mode {
	AD4692_CNV_CLOCK_MODE,
	AD4692_CNV_BURST_MODE,
	AD4692_AUTONOMOUS_MODE,
	AD4692_SPI_BURST_MODE,
	AD4692_MANUAL_MODE,
};

enum ad4692_gpio_mode {
	AD4692_HIGH_Z,
	AD4692_DIGITAL_OUTPUT_LOW,
	AD4692_DIGITAL_OUTPUT_HIGH,
	AD4692_DIGITAL_INPUT,
	AD4692_ADC_BUSY,
	AD4692_SEQ_DONE,
	AD4692_DATA_READY,
	AD4692_ACC_OVR_ERROR,
	AD4692_ACC_SAT_ERROR,
};

enum ad4692_int_osc_freq {
	AD4692_OSC_1MHZ = 0,
	AD4692_OSC_500KHZ,
	AD4692_OSC_400KHZ,
	AD4692_OSC_250KHZ,
	AD4692_OSC_200KHZ,
	AD4692_OSC_167KHZ,
	AD4692_OSC_133KHZ,
	AD4692_OSC_125KHZ,
	AD4692_OSC_100KHZ,
	AD4692_OSC_50KHZ,
	AD4692_OSC_25KHZ,
	AD4692_OSC_12P5KHZ,
	AD4692_OSC_10KHZ,
	AD4692_OSC_5KHZ,
	AD4692_OSC_2P5KHZ,
	AD4692_OSC_1P25KHZ,
};

enum ad4692_ref_ctrl {
	AD4692_VREF_2P5 = 0,
	AD4692_VREF_3P0,
	AD4692_VREF_3P3,
	AD4692_VREF_4P096,
	AD4692_VREF_5P0,
};

static int ad4692_int_osc_val[] = {
	[AD4692_OSC_1MHZ] = 1000000,
	[AD4692_OSC_500KHZ] = 500000,
	[AD4692_OSC_400KHZ] = 400000,
	[AD4692_OSC_250KHZ] = 250000,
	[AD4692_OSC_200KHZ] = 200000,
	[AD4692_OSC_167KHZ] = 167000,
	[AD4692_OSC_133KHZ] = 133000,
	[AD4692_OSC_125KHZ] = 125000,
	[AD4692_OSC_100KHZ] = 100000,
	[AD4692_OSC_50KHZ] = 50000,
	[AD4692_OSC_25KHZ] = 25000,
	[AD4692_OSC_12P5KHZ] = 12500,
	[AD4692_OSC_10KHZ] = 10000,
	[AD4692_OSC_5KHZ] = 5000,
	[AD4692_OSC_2P5KHZ] = 2500,
	[AD4692_OSC_1P25KHZ] = 1250,
};
struct ad4692_chip_info {
	struct iio_chan_spec *channels;
	const char *name;
	unsigned int num_channels;
	unsigned int resolution;
	unsigned int max_rate;
};

static const struct iio_chan_spec ad4691_channels[] = {
	AD4692_CHANNELS(18, 32)
};

static const struct iio_chan_spec ad4692_channels[] = {
	AD4692_CHANNELS(16, 16)
};

static const struct iio_chan_spec ad4693_channels[] = {
	AD4694_CHANNELS(14, 16)
};

static const struct iio_chan_spec ad4694_channels[] = {
	AD4694_CHANNELS(20, 32)
};

static const struct ad4692_chip_info ad4692_chips[] =  {
	[ID_AD4692] = {
		.channels = ad4692_channels,
		.name = "ad4692",
		.num_channels = ARRAY_SIZE(ad4692_channels),
		.resolution = 16,
		.max_rate = 1000000,
	},
	[ID_AD4691] = {
		.channels = ad4691_channels,
		.name = "ad4691",
		.num_channels = ARRAY_SIZE(ad4692_channels),
		.resolution = 18,
		.max_rate = 500000,
	},
	[ID_AD4694] = {
		.channels = ad4694_channels,
		.name = "ad4694",
		.num_channels = ARRAY_SIZE(ad4694_channels),
		.resolution = 20,
		.max_rate = 1000000,
	},
	[ID_AD4693] = {
		.channels = ad4693_channels,
		.name = "ad4693",
		.num_channels = ARRAY_SIZE(ad4694_channels),
		.resolution = 14,
		.max_rate = 500000,
	},
};

struct ad4692_state {
	const struct ad4692_chip_info	*chip;
	struct spi_device		*spi;

	unsigned long			ref_clk_rate;
	struct pwm_device		*conv_trigger;

	struct gpio_desc		*reset_gpio;
	struct gpio_desc		*gpio_0;
	struct gpio_desc		*gpio_1;
	struct gpio_desc		*gpio_2;
	struct gpio_desc		*gpio_3;

	struct regulator_bulk_data	regulators[AD4692_NUM_REGULATORS];

	struct iio_trigger		*trig;

	enum ad4692_adc_mode		adc_mode;

	int irq;
	int vio;
	int vref;
	u64				cnv_period;
	/*
	 * Synchronize access to members of the driver state, and ensure
	 * atomicity of consecutive SPI operations.
	 */
	struct mutex			lock;

	struct spi_message		msg;
	struct spi_transfer		xfer;
	int max_rate;

	u8 tx_data[3];
	u8 rx_data[3];
};

static void ad4692_disable_regulators(void *data)
{
	struct ad4692_state *st = data;

	regulator_bulk_disable(AD4692_NUM_REGULATORS, st->regulators);
}

static void ad4692_disable_pwm(void *data)
{
	struct ad4692_state *st = data;

	pwm_disable(st->conv_trigger);
}

static int ad4692_regulators_get(struct ad4692_state *st)
{
	struct device *dev = &st->spi->dev;
	struct regulator *ref;
	int ret;

	st->regulators[0].supply = "vio";

	ret = devm_regulator_bulk_get(dev, AD4692_NUM_REGULATORS,
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get VIO regulator\n");

	ret = regulator_bulk_enable(AD4692_NUM_REGULATORS, st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad4692_disable_regulators, st);
	if (ret)
		return ret;

	st->vio = regulator_get_voltage(st->regulators[0].consumer);

	ref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(ref)) {
		if (PTR_ERR(ref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vref regulator");

		/* Internal REFIN must be used if optional REF isn't used. */
		ref = devm_regulator_get(dev, "vrefin");
		if (IS_ERR(ref))
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vrefin regulator");
	}

	ret = regulator_enable(ref);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to enable specified ref supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, ad4692_disable_regulators, ref);
	if (ret)
		return ret;

	st->vref = regulator_get_voltage(ref);
	if (st->vref < AD4692_VREF_MIN || st->vref > AD4692_VREF_MAX)
		return dev_err_probe(dev, -EINVAL, "vref(%d) must be under [%u %u]\n",
				     st->vref, AD4692_VREF_MIN, AD4692_VREF_MAX);

	return 0;
}

int ad4692_spi_read(struct spi_device *spi, unsigned int reg, unsigned int *val)
{
	unsigned char buf[6];
	int ret;

	buf[0] = (reg >> 8) | 0x80;
	buf[1] = reg & 0xFF;

	switch (reg) {
	case 0 ... AD4692_INTERNAL_OSCILLATOR:
	case AD4692_SPARE_CONTROL ... AD4692_ACC_SAT_OVR_REG(15):
		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
		*val = buf[2];
		break;
	case AD4692_STD_SEQ_CONFIG:
	case AD4692_IN_ACC_16B(0) ... AD4692_IN_ACC_16B(15):
		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 2);
		*val = get_unaligned_le16(&buf[2]);
		break;
	case AD4692_IN_ACC_16B_STATUS(0) ... AD4692_IN_ACC_16B_STATUS(15):
		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 3);
		*val = get_unaligned_be24(&buf[2]);
		break;
	case AD4692_IN_ACC_24B(0) ... AD4692_IN_ACC_24B(15):
		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 3);
		*val = get_unaligned_le24(&buf[2]);
		break;
	case AD4692_IN_ACC_24B_STATUS(0) ... AD4692_IN_ACC_24B_STATUS(15):
		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 4);
		*val = get_unaligned_be32(&buf[2]);
		break;
	default: /* Test Registers Acces. */
		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
		*val = buf[2];
		break;
	}

	if (ret)
		return ret;

	printk("reg: 0x%x, val: 0x%x\n", reg, *val);

	return 0;
}

int ad4692_spi_write(struct spi_device *spi, unsigned int reg, unsigned int val)
{
	unsigned char buf[4];
	int ret;

	buf[0] = (reg >> 8);
	buf[1] = reg & 0xFF;

	switch (reg) {
	case 0 ... AD4692_INTERNAL_OSCILLATOR:
	case AD4692_SPARE_CONTROL ... AD4692_GPIO_MODE2_REG:
		if (val > 0xFF)
			return -EINVAL;
		buf[2] = val;

		ret = spi_write(spi, buf, 3);
		break;
	case AD4692_STD_SEQ_CONFIG:
		if (val > 0xFFFF)
			return -EINVAL;
		put_unaligned_le16(val, &buf[2]);

		ret = spi_write(spi, buf, 4);
		break;
	default: /* Test Register Access. */
		if (val > 0xFF)
			return -EINVAL;
		buf[2] = val;

		ret = spi_write(spi, buf, 3);
		break;
	}

	if (ret)
		return ret;

	printk("reg: 0x%x, val: 0x%x\n", reg, val);

	return 0;
}

int ad4692_spi_update_bits(struct spi_device *spi, unsigned int reg,
			   unsigned int mask, unsigned int val)
{
	unsigned int tmp, ret;

	ret = ad4692_spi_read(spi, reg, &tmp);
	if (ret < 0)
		return ret;

	tmp = tmp & ~mask;
	tmp |= val & mask;

	return ad4692_spi_write(spi, reg, tmp);
}

// Used for Manual Mode, find out more details in Manual Mode.
static int ad4692_transfer(struct iio_dev *indio_dev, int command, int *val)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer = {
		.speed_hz = 100000,
		.tx_buf = st->tx_data,
		.rx_buf = st->rx_data,
		.len = 3,
	};
	int ret;

	memcpy(st->tx_data, &command, 3);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(val, st->rx_data, 3);

	return 0;
}

static int ad4692_get_sampling_freq(struct ad4692_state *st)
{
	unsigned int val;
	int ret;

	switch (st->adc_mode) {
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_MANUAL_MODE:
		return DIV_ROUND_CLOSEST_ULL(NANO, pwm_get_period(st->conv_trigger));
	case AD4692_CNV_BURST_MODE:
	case AD4692_AUTONOMOUS_MODE:
	case AD4692_SPI_BURST_MODE:
		ret = ad4692_spi_read(st->spi, AD4692_INTERNAL_OSCILLATOR, &val);
		if (ret)
			return ret;

		return ad4692_int_osc_val[val];
	default:
		return -EINVAL;
	}
}

/* NOTE: Only enabled/disabled should be used instead of DUTY_CYCLE in final
 * 	 rev.
 */
static int __ad4692_set_sampling_freq(struct ad4692_state *st, int freq)
{
	unsigned long long target, ref_clk_period_ns;
	struct pwm_state cnv_state;

	pwm_init_state(st->conv_trigger, &cnv_state);

	freq = clamp(freq, 0, st->chip->max_rate);
	target = DIV_ROUND_CLOSEST_ULL(st->ref_clk_rate, freq);
	ref_clk_period_ns = DIV_ROUND_CLOSEST_ULL(NANO, st->ref_clk_rate);
	st->cnv_period = ref_clk_period_ns * target;
	cnv_state.period = ref_clk_period_ns * target;

	/* In case of CNV Burst Mode, we only need one pulse so we assure that
	 * the perid for CNV is larger than the maximum period of the internal
	 * oscillator as where all the channels are enabled.
	 */
	if (st->adc_mode == AD4692_CNV_BURST_MODE)
		cnv_state.period *= 16;

	cnv_state.duty_cycle = 0;
	cnv_state.time_unit = PWM_UNIT_NSEC;
	cnv_state.enabled = true;

	return pwm_apply_state(st->conv_trigger, &cnv_state);
}

static int ad4692_pwm_get(struct spi_device *spi, struct ad4692_state *st)
{
	struct clk *ref_clk;
	int ret;

	ref_clk = devm_clk_get_enabled(&spi->dev, NULL);
	if (IS_ERR(ref_clk))
		return PTR_ERR(ref_clk);

	st->ref_clk_rate = clk_get_rate(ref_clk);

	st->conv_trigger = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(st->conv_trigger)) {
		return dev_err_probe(&spi->dev, PTR_ERR(st->conv_trigger),
				     "Failed to get cnv pwm\n");
	}

	ret = devm_add_action_or_reset(&spi->dev, ad4692_disable_pwm,
				       st->conv_trigger);
	if (ret)
		return ret;

	return __ad4692_set_sampling_freq(st, st->chip->max_rate);
}

static int ad4692_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret, i;

	switch (st->adc_mode) {
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_MANUAL_MODE:
		if (!st->conv_trigger)
			return -ENODEV;

		if (!freq || freq > st->chip->max_rate)
			return -EINVAL;

		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = __ad4692_set_sampling_freq(st, freq);
		iio_device_release_direct_mode(indio_dev);

		return ret;
	case AD4692_CNV_BURST_MODE:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		i = find_closest_descending(freq, ad4692_int_osc_val, 16);
		ret = ad4692_spi_write(st->spi, AD4692_INTERNAL_OSCILLATOR, i);
		if (ret) {
			iio_device_release_direct_mode(indio_dev);
			return ret;
		}
		ret = __ad4692_set_sampling_freq(st, ad4692_int_osc_val[i]);
		iio_device_release_direct_mode(indio_dev);

		return ret;
	case AD4692_AUTONOMOUS_MODE:
	case AD4692_SPI_BURST_MODE:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		i = find_closest_descending(freq, ad4692_int_osc_val, 16);
		ret = ad4692_spi_write(st->spi, AD4692_INTERNAL_OSCILLATOR, i);

		iio_device_release_direct_mode(indio_dev);
		return ret;
	default:
		return -EINVAL;
	}
}

static int ad4692_sampling_enable(const struct ad4692_state *st, bool enable)
{
	struct pwm_state conv_state;

	pwm_get_state(st->conv_trigger, &conv_state);
	conv_state.duty_cycle = enable ? AD4692_CNV_DUTY_CYCLE_NS : 0;

	return pwm_apply_state(st->conv_trigger, &conv_state);
}

static int ad4692_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		switch (st->adc_mode) {
		case AD4692_CNV_CLOCK_MODE: /* Uses SPI Engine. */
			break;
		case AD4692_CNV_BURST_MODE:
			ret = ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG, BIT(chan->channel));
			if (ret)
				return ret;

			ret = ad4692_sampling_enable(st, true);
			if (ret)
				return ret;

			ndelay(2 * AD4692_OSC_PERIOD_NS(ad4692_get_sampling_freq(st)) + AD4692_CONV_PERIOD_NS);

			ret = ad4692_sampling_enable(st, false);
			if (ret)
				goto done;

			ret = ad4692_spi_read(st->spi, st->chip->resolution > 16 ? AD4692_IN_ACC_24B(chan->channel) : AD4692_IN_ACC_16B(chan->channel), val);

			break;
		case AD4692_AUTONOMOUS_MODE:
			ret = ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG, BIT(chan->channel));
			if (ret)
				return ret;

			ret = ad4692_spi_update_bits(st->spi, AD4692_CONV_START_REG,
						     AD4692_CONV_START_MASK,
						     FIELD_PREP(AD4692_CONV_START_MASK, 1));
			if (ret)
				return ret;

			ndelay(2 * AD4692_OSC_PERIOD_NS(ad4692_get_sampling_freq(st)) + AD4692_CONV_PERIOD_NS);

			ret = ad4692_spi_update_bits(st->spi, AD4692_CONV_START_REG,
						     AD4692_CONV_START_MASK,
						     FIELD_PREP(AD4692_CONV_START_MASK, 0));
			if (ret)
				goto done;

			ret = ad4692_spi_read(st->spi, st->chip->resolution > 16 ? AD4692_IN_ACC_24B(chan->channel) : AD4692_IN_ACC_16B(chan->channel), val);

			break;
		case AD4692_SPI_BURST_MODE:
			ret = ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG, BIT(chan->channel));
			if (ret)
				return ret;

			ret = ad4692_spi_update_bits(st->spi, AD4692_CONV_START_REG,
						     AD4692_CONV_START_MASK,
						     FIELD_PREP(AD4692_CONV_START_MASK, 1));
			if (ret)
				return ret;

			ndelay(2 * AD4692_OSC_PERIOD_NS(ad4692_get_sampling_freq(st)) + AD4692_CONV_PERIOD_NS);

			ret = ad4692_spi_read(st->spi, st->chip->resolution > 16 ? AD4692_IN_ACC_24B(chan->channel) : AD4692_IN_ACC_16B(chan->channel), val);

			break;
		case AD4692_MANUAL_MODE: /* Uses SPI Engine. */
			break;
		default:
			return -EINVAL;
		}

done:
		iio_device_release_direct_mode(indio_dev);

		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad4692_get_sampling_freq(st);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4692_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4692_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ad4692_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	const struct ad4692_state *st = iio_priv(indio_dev);

	if (st->adc_mode == AD4692_MANUAL_MODE)
		return -EOPNOTSUPP;

	if (readval)
		return ad4692_spi_read(st->spi, reg, readval);

	return ad4692_spi_write(st->spi, reg, writeval);
}

static int ad4692_config(struct ad4692_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int reg_val;
	u32 mode;
	int ret;

	ret = device_property_read_u32(dev, "adi,spi-mode", &mode);
	if (ret)
		return dev_err_probe(dev, -EINVAL, "Could not find SPI mode\n");

	if (mode > AD4692_MAX_ADC_MODE)
		return dev_err_probe(dev, -EINVAL, "Invalid SPI mode(%u)\n", mode);

	st->adc_mode = mode;

	if (st->adc_mode != AD4692_AUTONOMOUS_MODE && st->adc_mode != AD4692_SPI_BURST_MODE) {
		if (device_property_present(dev, "pwms")) {
			ret = ad4692_pwm_get(st->spi, st);
			if (ret)
				return ret;
		}
		else
			return -EIO;
	}

	printk("pwm ok\n");

	/* Configure as address ASCENDING, this may be forbidden in final rev
	 * therefore a way of accessing registers in a DESCENDING address mode
	 * should be considered.
	 */
	ret = ad4692_spi_update_bits(st->spi, AD4692_INTERFACE_CONFIG_A_REG,
				     AD4692_ADDR_ASCENSION,
				     FIELD_PREP(AD4692_ADDR_ASCENSION, 1));
	if (ret) {
		printk("SPI Core Config FAIL!.\n");
		return ret;
	}

	/* Clear STATUS register by reading from the STATUS register. */
	ret = ad4692_spi_read(st->spi, AD4692_STATUS_REG, &reg_val);
	if (ret)
		return ret;

	/* Perform a state reset on the channels at start-up. */
	ret = ad4692_spi_write(st->spi, AD4692_STATE_RESET_REG, 0x01);
	if (ret)
		return ret;

	switch (st->vref) {
	case AD4692_VREF_MIN ... 2750000:
		ret = ad4692_spi_write(st->spi, AD4692_REFERENCE_CONTROL,
				       FIELD_PREP(AD4692_REF_CTRL_MASK,
						  AD4692_VREF_2P5));
		break;
	case 2750001 ... 3250000:
		ret = ad4692_spi_write(st->spi, AD4692_REFERENCE_CONTROL,
				       FIELD_PREP(AD4692_REF_CTRL_MASK,
						  AD4692_VREF_3P0));
		break;
	case 3250001 ... 3750000:
		ret = ad4692_spi_write(st->spi, AD4692_REFERENCE_CONTROL,
				       FIELD_PREP(AD4692_REF_CTRL_MASK,
						  AD4692_VREF_3P3));
		break;
	case 3750001 ... 4500000:
		ret = ad4692_spi_write(st->spi, AD4692_REFERENCE_CONTROL,
				       FIELD_PREP(AD4692_REF_CTRL_MASK,
						  AD4692_VREF_4P096));
		break;
	case 4500001 ... AD4692_VREF_MAX:
		ret = ad4692_spi_write(st->spi, AD4692_REFERENCE_CONTROL,
				       FIELD_PREP(AD4692_REF_CTRL_MASK,
						  AD4692_VREF_5P0));
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	switch (st->adc_mode) {
	case AD4692_CNV_CLOCK_MODE: /* SPI Engine. */
		break;
	case AD4692_CNV_BURST_MODE:
		ret = ad4692_spi_write(st->spi, AD4692_ADC_OPERATION_REG, mode | AD4692_DATA_STOP_STATE);
		if (ret)
			return ret;

		/* Configure GPI0 as DATA_READY and use as interrupt for
		 * reading.
		 */
		ret = ad4692_spi_update_bits(st->spi, AD4692_GPIO_MODE1_REG,
					     AD4692_GPIO_MODE_MASK(0),
					     FIELD_PREP(AD4692_GPIO_MODE_MASK(0), AD4692_DATA_READY));
		if (ret)
			return ret;
		break;
	case AD4692_AUTONOMOUS_MODE:
		ret = ad4692_spi_write(st->spi, AD4692_ADC_OPERATION_REG, mode);
		if (ret)
			return ret;

		/* Configure GPI0 as DATA_READY and use as interrupt for
		 * reading.
		 */
		ret = ad4692_spi_update_bits(st->spi, AD4692_GPIO_MODE1_REG,
					     AD4692_GPIO_MODE_MASK(0),
					     FIELD_PREP(AD4692_GPIO_MODE_MASK(0), AD4692_DATA_READY));
		if (ret)
			return ret;
		break;
	case AD4692_SPI_BURST_MODE:
		ret = ad4692_spi_write(st->spi, AD4692_ADC_OPERATION_REG, mode | AD4692_DATA_STOP_STATE);
		if (ret)
			return ret;

		/* Configure GPI0 as DATA_READY and use as interrupt for
		 * reading.
		 */
		ret = ad4692_spi_update_bits(st->spi, AD4692_GPIO_MODE1_REG,
					     AD4692_GPIO_MODE_MASK(0),
					     FIELD_PREP(AD4692_GPIO_MODE_MASK(0), AD4692_DATA_READY));
		if (ret)
			return ret;
		break;
	case AD4692_MANUAL_MODE: /* SPI Engine. */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad4692_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret, i;

	switch (st->adc_mode) {
	case AD4692_AUTONOMOUS_MODE:
	case AD4692_SPI_BURST_MODE:
		ret = ad4692_spi_write(st->spi, AD4692_ACC_MASK1_REG, ~(*indio_dev->active_scan_mask) & 0xFF);
		if (ret)
			return ret;

		ret = ad4692_spi_write(st->spi, AD4692_ACC_MASK2_REG, ~(*indio_dev->active_scan_mask >> 8) & 0xFF);
		if (ret)
			return ret;

		ret = ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG,
				       *indio_dev->active_scan_mask);
		if (ret)
			return ret;

		for (i = 0; i < st->chip->num_channels; i++)
			if (BIT(i) & *indio_dev->active_scan_mask) {
				ret = ad4692_spi_write(st->spi, AD4692_ACC_COUNT_LIMIT(i), hweight16(*indio_dev->active_scan_mask));
				if (ret)
					return ret;
			}

		return ad4692_spi_write(st->spi, AD4692_CONV_START_REG, AD4692_CONV_START);
	case AD4692_CNV_BURST_MODE:
		ret = ad4692_spi_write(st->spi, AD4692_ACC_MASK1_REG, ~(*indio_dev->active_scan_mask) & 0xFF);
		if (ret)
			return ret;

		ret = ad4692_spi_write(st->spi, AD4692_ACC_MASK2_REG, ~(*indio_dev->active_scan_mask >> 8) & 0xFF);
		if (ret)
			return ret;

		ret = ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG,
				       *indio_dev->active_scan_mask);
		if (ret)
			return ret;

		for (i = 0; i < st->chip->num_channels; i++)
			if (BIT(i) & *indio_dev->active_scan_mask) {
				ret = ad4692_spi_write(st->spi, AD4692_ACC_COUNT_LIMIT(i), hweight16(*indio_dev->active_scan_mask));
				if (ret)
					return ret;
			}

		return ad4692_sampling_enable(st, true);
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_MANUAL_MODE:
		return ad4692_sampling_enable(st, true);
	default:
		return -EOPNOTSUPP;
	}
}

static int ad4692_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4692_state *st = iio_priv(indio_dev);

	switch (st->adc_mode) {
	case AD4692_AUTONOMOUS_MODE:
		ad4692_spi_write(st->spi, AD4692_CONV_START_REG,
				 AD4692_CONV_STOP);

		ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG,
				 AD4692_SEQ_ALL_CHANNELS_OFF);
		break;
	case AD4692_CNV_BURST_MODE:
		ad4692_sampling_enable(st, false);

		ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG,
				 AD4692_SEQ_ALL_CHANNELS_OFF);
		break;
	case AD4692_SPI_BURST_MODE:
		ad4692_spi_write(st->spi, AD4692_STD_SEQ_CONFIG,
				 AD4692_SEQ_ALL_CHANNELS_OFF);
		break;
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_MANUAL_MODE:
		ad4692_sampling_enable(st, false);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ad4692_spi_write(st->spi, AD4692_STATE_RESET_REG,
				AD4692_STATE_RESET_ALL);
}

static const struct iio_buffer_setup_ops ad4692_buffer_setup_ops = {
	.postenable = &ad4692_buffer_postenable,
	.postdisable = &ad4692_buffer_postdisable,
};

static int ad4692_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	struct ad4692_state *st = iio_trigger_get_drvdata(trig);

	if (enable)
		enable_irq(st->irq);
	else
		disable_irq(st->irq);

	return 0;
}

static const struct iio_trigger_ops ad4692_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
	.set_trigger_state = &ad4692_set_trigger_state,
};

static irqreturn_t ad4692_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4692_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret, i;

	/* STOP Conversion if needed. */
	switch (st->adc_mode) {
	case AD4692_CNV_BURST_MODE:
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_MANUAL_MODE:
		ad4692_sampling_enable(st, false);
		break;
	case AD4692_AUTONOMOUS_MODE:
		ad4692_spi_write(st->spi, AD4692_CONV_START_REG,
				 AD4692_CONV_STOP);
		break;
	case AD4692_SPI_BURST_MODE: /* CONV_START bit clears itself. */
		break;
	default:
		return -EINVAL;
	}

	switch (st->adc_mode) {
	case AD4692_CNV_CLOCK_MODE: /* SPI Engine. */
		break;
	case AD4692_CNV_BURST_MODE:
		mutex_lock(&st->lock);

		for (i = 0; i < st->chip->num_channels - 1; i++) {
			if (BIT(i) & *indio_dev->active_scan_mask) {
				ret = ad4692_spi_read(st->spi, st->chip->resolution > 16 ? AD4692_IN_ACC_24B(i) : AD4692_IN_ACC_16B(i), &val);
				if (!ret)
					iio_push_to_buffers_with_timestamp(indio_dev, &val,
									   iio_get_time_ns(indio_dev));
				else
					goto done;
			}
		}

		iio_trigger_notify_done(indio_dev->trig);

		/* START next conversion. */
		ad4692_sampling_enable(st, true);

		mutex_unlock(&st->lock);

		return IRQ_HANDLED;
	case AD4692_AUTONOMOUS_MODE:
	case AD4692_SPI_BURST_MODE:
		mutex_lock(&st->lock);

		for (i = 0; i < st->chip->num_channels - 1; i++) {
			if (BIT(i) & *indio_dev->active_scan_mask) {
				ret = ad4692_spi_read(st->spi, st->chip->resolution > 16 ? AD4692_IN_ACC_24B(i) : AD4692_IN_ACC_16B(i), &val);
				if (!ret)
					iio_push_to_buffers_with_timestamp(indio_dev, &val,
									   iio_get_time_ns(indio_dev));
				else
					goto done;
			}
		}

		iio_trigger_notify_done(indio_dev->trig);

		/* START next conversion. */
		ad4692_spi_write(st->spi, AD4692_CONV_START_REG,
				 AD4692_CONV_START);

		mutex_unlock(&st->lock);

		return IRQ_HANDLED;
	case AD4692_MANUAL_MODE:
		break;
	default:
		return -EINVAL;
	}

done:
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}

static const struct iio_info ad4692_info = {
	.read_raw = &ad4692_read_raw,
	.write_raw = &ad4692_write_raw,
	.debugfs_reg_access = &ad4692_reg_access,
};

static const struct spi_device_id ad4692_id[] = {
	{ "ad4692", (kernel_ulong_t)&ad4692_chips[ID_AD4692] },
	{ "ad4691", (kernel_ulong_t)&ad4692_chips[ID_AD4691] },
	{ "ad4694", (kernel_ulong_t)&ad4692_chips[ID_AD4694] },
	{ "ad4693", (kernel_ulong_t)&ad4692_chips[ID_AD4693] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4692_id);

/* NOTE: Maybe handle RESET diffferent way if there is one. */
static int ad4692_gpio_setup(struct ad4692_state *st)
{
	struct device *dev = &st->spi->dev;

	switch (st->adc_mode) {
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_MANUAL_MODE:
		st->gpio_0 = devm_gpiod_get_optional(dev, "gpiomode0", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		st->gpio_1 = devm_gpiod_get_optional(dev, "gpiomode1", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		st->gpio_2 = devm_gpiod_get_optional(dev, "gpiomode2", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		st->gpio_3 = devm_gpiod_get_optional(dev, "gpiomode3", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);
		break;
	case AD4692_CNV_BURST_MODE:
	case AD4692_AUTONOMOUS_MODE:
	case AD4692_SPI_BURST_MODE:
		/* For this modes, GPIO0 is mandatory in order to serve as
		 * DATA_READYb pin and later be used as interrupt for buffered
		 * readings.
		 */
		st->gpio_0 = devm_gpiod_get(dev, "gpiomode0", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		st->gpio_1 = devm_gpiod_get_optional(dev, "gpiomode1", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		st->gpio_2 = devm_gpiod_get_optional(dev, "gpiomode2", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		st->gpio_3 = devm_gpiod_get_optional(dev, "gpiomode3", GPIOD_IN);
		if (IS_ERR(st->gpio_0))
			return PTR_ERR(st->gpio_0);

		break;
	default:
		return -EINVAL;
	}

	st->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	/* Reset delay required from the datasheet of 3.2ms */
	fsleep(3200);
	gpiod_set_value_cansleep(st->reset_gpio, 0);

	return 0;
}

static int ad4692_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4692_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, indio_dev);
	st->chip = device_get_match_data(dev);
	if (!st->chip) {
		st->chip = (void *)spi_get_device_id(spi)->driver_data;
		if (!st->chip)
			return dev_err_probe(dev, -ENODEV,
					     "Could not find chip info data\n");
	}

	indio_dev->name = st->chip->name;
	indio_dev->info = &ad4692_info;
	indio_dev->channels = st->chip->channels;
	indio_dev->num_channels = st->chip->num_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad4692_regulators_get(st);
	if (ret)
		return ret;

	ret = ad4692_gpio_setup(st);
	if (ret)
		return ret;

	ret = ad4692_config(st);
	if (ret)
		return ret;

	printk("config\n");

	mutex_init(&st->lock);

	st->irq = fwnode_irq_get_byname(dev_fwnode(dev), "DRDY");
	if (st->irq > 0) {
		st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
						indio_dev->name, iio_device_id(indio_dev));
		if (!st->trig)
			return -ENOMEM;

		st->trig->ops= &ad4692_trigger_ops;
		iio_trigger_set_drvdata(st->trig, st);
		ret = devm_iio_trigger_register(&spi->dev, st->trig);
		if (ret) {
			dev_err(&spi->dev, "IIO trigger register failed\n");
			return ret;
		}

		indio_dev->trig = iio_trigger_get(st->trig);

		switch (st->adc_mode) {
		case AD4692_CNV_CLOCK_MODE:
		case AD4692_MANUAL_MODE: /* SPI Engine interrupt may be used. */
			break;
		case AD4692_CNV_BURST_MODE:
		case AD4692_AUTONOMOUS_MODE:
		case AD4692_SPI_BURST_MODE:
			ret = devm_request_irq(&spi->dev, st->irq,
					&iio_trigger_generic_data_rdy_poll,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					indio_dev->name, st->trig);
			if (ret)
				return dev_err_probe(&spi->dev, ret, "request irq %d failed\n",
						st->irq);
			break;
		default:
			return -EINVAL;
		}
	}

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      NULL,
					      ad4692_trigger_handler,
					      &ad4692_buffer_setup_ops);
	if (ret) {
		dev_err(&spi->dev, "IIO triggered buffer setup failed\n");
		return ret;
	}

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad4692_of_match[] = {
	{ .compatible = "adi,ad4692", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4692]},
	{ .compatible = "adi,ad4691", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4691]},
	{ .compatible = "adi,ad4694", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4694]},
	{ .compatible = "adi,ad4693", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4693]},
	{},
};
MODULE_DEVICE_TABLE(of, ad4692_of_match);

static struct spi_driver ad4692_driver = {
	.driver = {
		.name = "ad4692",
		.of_match_table = ad4692_of_match,
	},
	.probe = ad4692_probe,
	.id_table = ad4692_id,
};
module_spi_driver(ad4692_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4692 ADC Driver");
MODULE_LICENSE("GPL v2");
