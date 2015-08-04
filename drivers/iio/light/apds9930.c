/*
 * This is a driver for Avago APDS 9930 ALS sensor chip. It
 * is inspired from drivers/iio/misc/apds9930.c to use IIO.
 * The datasheet for this device can be found at:
 *	http://www.avagotech.com/docs/AV02-3190EN
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * TODO: runtime pm support
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/bitops.h>

#include <linux/iio/types.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>

#include <linux/gpio/consumer.h>

#define APDS9930_DRIVER_NAME	"apds9930"
#define APDS9930_IRQ_NAME	"apds9930_irq"
#define APDS9930_GPIO_NAME	"apds9930_gpio"

#define APDS9930_INIT_SLEEP	5 /* sleep for 5 ms before issuing commands */

/* Register set (rw = read/write, r = read, w = write) */
#define APDS9930_ENABLE_REG	0x00	/* rw-Enable of states and interrupts */
#define APDS9930_ATIME_REG	0x01	/* rw-ALS ADC time*/
#define APDS9930_PTIME_REG	0x02	/* rw-Proximity ADC time */
#define APDS9930_WTIME_REG	0x03	/* rw-Wait time */
#define APDS9930_AILTL_REG	0x04	/* rw-ALS interrupt low threshold low
					 * byte
					 */
#define APDS9930_AIHTL_REG	0x06	/* rw-ALS interrupt high threshold low
					 * byte
					 */
#define APDS9930_PILTL_REG	0x08	/* rw-Proximity interrupt low threshold
					 * low byte
					 */
#define APDS9930_PIHTL_REG	0x0A	/* rw-Proximity interrupt high threshold
					 * low byte
					 */
#define APDS9930_PERS_REG	0x0C	/* rw-Interrupt persistance filters */
#define APDS9930_CONFIG_REG	0x0D	/* rw-Configuration */
#define APDS9930_PPULSE_REG	0x0E	/* rw-Proximity pulse control */
#define APDS9930_CONTROL_REG	0x0F	/* rw-Gain control register */
#define APDS9930_ID_REG		0x12	/* r-Device ID */
#define APDS9930_STATUS_REG	0x13	/* r-Device status */
#define APDS9930_CDATAL_REG	0x14	/* r-Ch0 ADC low data register */
#define APDS9930_CDATAH_REG	0x15	/* r-Ch0 ADC high data register */
#define APDS9930_IRDATAL_REG	0x16	/* r-Ch1 ADC low data register */
#define APDS9930_IRDATAH_REG	0x17	/* r-Ch1 ADC high data register */
#define APDS9930_PDATAL_REG	0x18	/* r-Proximity ADC low data register */
#define APDS9930_POFFSET_REG	0x1E	/* rw-Proximity offset register */

/* Useful bits per register */

/* Enable register */
#define APDS9930_SAI	BIT(6)	/* Sleep after interrupt */
#define APDS9930_PIEN	BIT(5)	/* Proximity interrupt enable */
#define APDS9930_AIEN	BIT(4)	/* ALS interrupt enable */
#define APDS9930_WEN	BIT(3)	/* Wait enable */
#define APDS9930_PEN	BIT(2)	/* Proximity enable */
#define APDS9930_AEN	BIT(1)	/* ALS enable */
#define APDS9930_PON	BIT(0)	/* Power ON */

/* Persistance register */
#define APDS9930_PPERS_SHIFT	4	/* Proximity interrupt persistance */
#define APDS9930_APERS_SHIFT	0	/* Interrupt persistance */

/* Configuration register */
#define APDS9930_AGL_SHIFT	2	/* ALS gain level */
#define APDS9930_WLONG_SHIFT	1	/* Wait long */
#define APDS9930_PDL_SHIFT	0	/* Proximity drive level */

/* Control register */
#define APDS9930_PDRIVE_SHIFT	6	/* LED drive strength */
#define APDS9930_PDIODE_SHIFT	4	/* Proximity diode select */
#define APDS9930_PGAIN_SHIFT	2	/* Proximity gain control */
#define APDS9930_AGAIN_SHIFT	0	/* ALS gain control */

/* Device ID - possible values */
#define APDS9930_ID	0x39

/* Status register */
#define APDS9930_PSAT	BIT(6)	/* Proximity saturation */
#define APDS9930_PINT	BIT(5)	/* Proximity interrupt */
#define APDS9930_AINT	BIT(4)	/* ALS interrupt */
#define APDS9930_PVALID	BIT(1)	/* PS valid */
#define APDS9930_AVALID	BIT(0)	/* ALS valid */

/* I2C command register - fields and possible field values */
#define APDS9930_CMD_TYPE_SHIFT		5
#define APDS9930_CMD_ADDR_SHIFT		0
/* CMD bit value must be set to 1 when addressing command register */
#define APDS9930_CMD_SELECT		0x80
/* TYPE possible values */
#define APDS9930_CMD_TYPE_RB		0		/* Repeated byte */
#define APDS9930_CMD_TYPE_AUTO_INC	BIT(5)		/* Auto-increment */
#define APDS9930_CMD_TYPE_SPECIAL_FUNC	GENMASK(6, 5)	/* Special function */
/* ADDR possible values */
#define APDS9930_CMD_TYPE_NONE		0x0	/* Normal - no action */
#define APDS9930_CMD_TYPE_PS		0x5	/* Proximity interrupt clear */
#define APDS9930_CMD_TYPE_ALS		0x6	/* ALS interrupt clear */
#define APDS9930_CMD_TYPE_BOTH		0x7	/* Both interrupt clear */
/* Clear masks */
#define APDS9930_CLEAR_CMD_TYPE_MASK	~(0x7 << APDS9930_CMD_TYPE_SHIFT)
/* Shortcut to clear and set CMD and TYPE fields */
#define APDS9930_CMD_REG_SETUP(reg, transaction_type) {		\
	reg &= APDS9930_CLEAR_CMD_TYPE_MASK;		\
	reg |= APDS9930_CMD_SELECT | transaction_type;	\
}

/* Default values for registers content */
#define APDS9930_DISABLE_ALL	0	/* Disable and powerdown */
#define APDS9930_ENABLE_ALL	0x07	/* Set all ALS & PS bits and power on */
#define APDS9930_DEF_ATIME	0xdb	/* 50 ms - ALSIT value in order to
					 * reject 50/60 Hz ripple; if higher
					 * resolution is needed, increase ALSIT
					 * with mutiples of 50
					 */
#define APDS9930_DEF_PTIME	0xff	/* 2.7 ms - min prox integration time */
#define APDS9930_DEF_WTIME	0xff	/* 2.7 ms - min wait time */
#define APDS9930_DEF_PPULSE	8	/* Min prox pulse count */
#define APDS9930_DEF_PDRIVE	0	/* 100 mA of LED power */
#define APDS9930_DEF_PDIODE	2	/* Ch1 diode (shifted value) */
#define APDS9930_DEF_PGAIN	2	/* 4 x proximity gain */
#define APDS9930_DEF_AGAIN	2	/* 16 x ALS gain */
#define APDS9930_DEF_WEN	8	/* Enable wait */
#define APDS9930_DEF_PEN	4	/* Enable proximity */
#define APDS9930_DEF_AEN	2	/* Enable ALS */
#define APDS9930_DEF_PON	1	/* Power ON */
#define APDS9930_DEF_APERS	3	/* Consecutive exceeding threshold
					 * cycles to trigger ALS interrupt
					 */
#define APDS9930_DEF_PPERS	3	/* Consecutive PS execeeding threshold
					 * cycles to trigger PS interrupt
					 */

/* Interrupt threshold defaults - setting low to maximum will trigger a first
 * interrupt to allow us to read the data registers status and properly set a
 * threshold value to match the current environment.
 *
 * The default high threshold is set only for consistency, since the registers
 * are checked against in order: first if the value of CDATA/PDATA is above LOW;
 * if so, trigger interupt and do not check the HIGH threshold.
 */
#define APDS9930_DEF_ALS_THRESH_LOW	0xffff
#define APDS9930_DEF_ALS_THRESH_HIGH	0x0
#define APDS9930_DEF_PS_THRESH_LOW	1023
#define APDS9930_DEF_PS_THRESH_HIGH	0

#define APDS9930_MIN_PS_THRESH		0	/* No object near the sensor */
#define APDS9930_MAX_PS_THRESH		1023	/* Object in close proximity */
#define APDS9930_DETECTION_PS_THRESH	600	/* Object detected near-by */

/* Default, open air, coefficients (for Lux computation) */
#define APDS9930_DEF_DF	52	/* Device factor */
/* Material-depending coefficients (set to open air values). They are scaled for
 * computation purposes by 100 (as stored in APDS9930_COEF_SCALE).
 */
#define APDS9930_DEF_LUX_DIVISION_SCALE	100
#define APDS9930_DEF_GA			49
#define APDS9930_DEF_B			186
#define APDS9930_DEF_C			74
#define APDS9930_DEF_D			129
#define APDS9930_COEF_SCALE		100

/* Possible ALS gain values (with AGL cleared) */
static const int apds9930_again_values[] = {1, 8, 16, 120};
#define APDS9930_MAX_AGAIN_INDEX	3
#define APDS9930_AGL_DIVISION_SCALE	6

/* Percentages from the maximum CH0 value that indicate the recorded CH0 data is
 * too low or too high - for AGAIN adapting purposes.
 */
#define APDS9930_CH0_HIGH_PERCENTAGE	90
#define APDS9930_CH0_LOW_PERCENTAGE	10

/* With how much (in percentages) must the CH0 value differ from one step to
 * another in order to consider a significant change in light. */
#define APDS9930_ALS_HYSTERESIS		20

/* DT or ACPI device properties names */
#define APDS9930_GA_PROP	"intel,ga"
#define APDS9930_COEF_B_PROP	"intel,coeff-B"
#define APDS9930_COEF_C_PROP	"intel,coeff-C"
#define APDS9930_COEF_D_PROP	"intel,coeff-D"
#define APDS9930_DF_PROP	"intel,df"
#define APDS9930_AGAIN_PROP	"intel,als-gain"
#define APDS9930_ATIME_PROP	"intel,atime"
#define APDS9930_PDRIVE_PROP	"intel,pdrive"
#define APDS9930_PPULSE_PROP	"intel,ppcount"

struct apds9930_coefficients {
	/* GA, B, C and D coefficients are scaled by 100 for computational
	   purposes. */
	int ga;
	int coef_a; /* This is 1, but needs to be set to a scaled value, thus if
		       we decide to change the scale, this coefficient must also
		       be changed. */
	int coef_b;
	int coef_c;
	int coef_d;
	int df;
};

struct apds9930_platform_data {
	/* Glass-influenced factors */
	struct apds9930_coefficients coefs;

	/* ALS platform data */
	u8 again;	/* AGAIN value (index in apds9930_again_values[]) */
	u8 atime;	/* ALS integration time */

	/* Proximity platform data */
	u8 pdrive;
	u8 ppulse;
};

struct apds9930_threshold {
	u16 low;
	u16 high;
};

struct apds9930_data {
	struct i2c_client	*client;
	struct mutex		mutex;

	/* Platform specific data */
	struct apds9930_platform_data	platform_data;

#define __coefs		platform_data.coefs
#define __again		platform_data.again
#define __atime		platform_data.atime
#define __pdrive	platform_data.pdrive
#define __ppulse	platform_data.ppulse

	u8	alsit;
	u16	ch0_max;		/* Maximum possible Ch0 data value */
	bool	agl_enabled;

	bool	als_intr_state;
	bool	ps_intr_state;
	struct apds9930_threshold	als_thresh;
	struct apds9930_threshold	ps_thresh;
};

/* Writes data to the register; the next i2c_write call will write to the same
 * register */
static inline int apds9930_write_byte(struct i2c_client *client, u8 reg,
				      u8 data)
{
	APDS9930_CMD_REG_SETUP(reg, APDS9930_CMD_TYPE_RB);

	return i2c_smbus_write_byte_data(client, reg, data);
}

/* Reads data from a register; the next i2c_read call will read from the same
 * address */
static inline int apds9930_read_byte(struct i2c_client *client, u8 reg,
				     u8 *data)
{
	int ret;

	APDS9930_CMD_REG_SETUP(reg, APDS9930_CMD_TYPE_RB);
	ret	= i2c_smbus_read_byte_data(client, reg);
	*data	= ret;

	return ret;
}

/* Writes 2 bytes at the given address */
static inline int apds9930_write_word(struct i2c_client *client, u8 reg,
				      u16 data)
{
	APDS9930_CMD_REG_SETUP(reg, APDS9930_CMD_TYPE_AUTO_INC);

	return i2c_smbus_write_word_data(client, reg, data);
}

/* Reads 2 bytes from the given address */
static inline int apds9930_read_word(struct i2c_client *client, u8 reg,
				     u16 *data)
{
	int ret;

	APDS9930_CMD_REG_SETUP(reg, APDS9930_CMD_TYPE_AUTO_INC);
	ret	= i2c_smbus_read_word_data(client, reg);
	*data	= ret;

	return ret;
}

/* ALSIT = 2.73ms * (256 - ATIME), 2.73 = 5591/(2^11) */
static inline u8 apds9930_atime_to_alsit(u8 atime_val)
{
	return (u8)(((256 - (u32)atime_val) * 5591) >> 11);
}

/* ATIME = 256 - ALSIT/2.73ms, 1/2.73 = 375/(2^10) */
static inline u8 apds9930_alsit_to_atime(u8 alsit_val)
{
	return (u8)(256 - (((u32)alsit_val * 375) >> 10));
}

/* Computes maximum Ch0 data when atime is the given one. */
static inline u16 apds9930_compute_max_ch0(u8 atime_val)
{
	return 1024 * (256 - (u32)atime_val) - 1;
}

/* Computes the ALS gain value for the next step and updates the current one */
static void apds9930_update_again(struct apds9930_data *data, u16 ch0)
{
	int current_index, next_index, err;

	/* Compute the value for the next measurement */
	current_index	= data->__again;
	next_index	= data->__again;

	/* CH0 data too high, try to lower the ALS gain if possible */
	if (ch0 >= (data->ch0_max * APDS9930_CH0_HIGH_PERCENTAGE) / 100) {
		if (next_index == 0 && !(data->agl_enabled)) {
			err = apds9930_write_byte(
						  data->client,
						  APDS9930_CONFIG_REG,
						  1 << APDS9930_AGL_SHIFT);
			if (!err)
				data->agl_enabled = true;
		} else if (next_index > 0) {
			next_index--;
		}
	}

	/* CH0 data too low, try to increase the ALS gain if possible */
	else if (ch0 <= (data->ch0_max * APDS9930_CH0_LOW_PERCENTAGE) / 100) {
		if (data->agl_enabled) {
			err = apds9930_write_byte(data->client,
						  APDS9930_CONFIG_REG,
						  0);
			if (!err)
				data->agl_enabled = false;
		} else if (next_index < APDS9930_MAX_AGAIN_INDEX) {
			next_index++;
		}
	}

	if (next_index != current_index) {
		/* Update data's index value */
		data->__again = next_index;

		/* Update AGAIN for the next reading */
		apds9930_write_byte(
				data->client,
				APDS9930_CONTROL_REG,
				data->__again << APDS9930_AGAIN_SHIFT |
				APDS9930_DEF_PGAIN << APDS9930_PGAIN_SHIFT |
				APDS9930_DEF_PDIODE << APDS9930_PDIODE_SHIFT |
				data->__pdrive << APDS9930_PDRIVE_SHIFT);
	}
}

/* Update thresholds so as to generate interrupts when the CH0 data changes
 * significantly since the last update; significantly is quantified by the
 * hysteresis factor: if the ALS state changes with more than hysteresis %,
 * update the thresholds.
 */
static void apds9930_update_als_thresholds(struct apds9930_data *data, u16 ch0)
{
	data->als_thresh.low	= (ch0*(100 - APDS9930_ALS_HYSTERESIS))/100;
	data->als_thresh.high	= min((ch0*(100 + APDS9930_ALS_HYSTERESIS))/100,
				      data->ch0_max);

	apds9930_write_word(data->client, APDS9930_AILTL_REG,
			    data->als_thresh.low);
	apds9930_write_word(data->client, APDS9930_AIHTL_REG,
			    data->als_thresh.high);
}

static void apds9930_update_ps_thresholds(struct apds9930_data *data, u16 ps)
{
	if (ps <= data->ps_thresh.low) {
		/* Near to far => set limits to detect when it comes close */
		data->ps_thresh.low	= APDS9930_MIN_PS_THRESH;
		data->ps_thresh.high	= APDS9930_DETECTION_PS_THRESH;
	} else if (ps >= data->ps_thresh.high) {
		/* Far to near => set limits to detect when it goes further */
		data->ps_thresh.low	= APDS9930_DETECTION_PS_THRESH;
		data->ps_thresh.high	= APDS9930_MAX_PS_THRESH;
	}

	/* Update thresholds */
	apds9930_write_word(data->client, APDS9930_PILTL_REG,
			    data->ps_thresh.low);
	apds9930_write_word(data->client, APDS9930_PIHTL_REG,
			    data->ps_thresh.high);
}

/* Having the channel 0 and channels 1's data, compute the Lux value */
static int apds9930_compute_lux(struct apds9930_data *data, u16 ch0, u16 ch1)
{
	long int iac1, iac2, alsit_val, again_val, tmp_iac;
	unsigned long int iac, lux;
	struct apds9930_coefficients cf;

	/* Lux equation */
	cf		= data->__coefs;
	iac1		= cf.coef_a * ch0 - cf.coef_b * ch1;
	iac2		= cf.coef_c * ch0 - cf.coef_d * ch1;
	tmp_iac         = max(iac1, iac2);
	iac             = (tmp_iac < 0) ? 0:(unsigned long)tmp_iac;
	alsit_val	= (int)(data->alsit);
	again_val	= apds9930_again_values[data->__again];
	lux		=
		DIV_ROUND_UP(DIV_ROUND_UP(iac, APDS9930_DEF_LUX_DIVISION_SCALE)
			     * cf.ga * cf.df,
		alsit_val * again_val * APDS9930_DEF_LUX_DIVISION_SCALE);

	return data->agl_enabled ? (APDS9930_AGL_DIVISION_SCALE * lux) : lux;
}

static int apds9930_enable_all(struct apds9930_data *data)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = apds9930_write_byte(data->client, APDS9930_ENABLE_REG,
				  APDS9930_ENABLE_ALL);
	if (ret < 0)
		goto err;

err:
	mutex_unlock(&data->mutex);

	return ret;
}

static int apds9930_disable_all(struct apds9930_data *data)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = apds9930_write_byte(data->client, APDS9930_ENABLE_REG,
				  APDS9930_DISABLE_ALL);
	if (ret < 0)
		goto err;

err:
	mutex_unlock(&data->mutex);

	return ret;
}

static int apds9930_chip_detect(struct apds9930_data *data)
{
	int ret;
	u8 id;

	ret = apds9930_read_byte(data->client, APDS9930_ID_REG, &id);
	if (ret < 0)
		return ret;

	if (id != APDS9930_ID)
		ret = -EINVAL;

	return ret;
}

/* Set the coefficients to those specified in properties if they exist,
 * otherwise to default values. */
static void apds9930_set_platform_data(struct apds9930_data *data)
{
	struct apds9930_platform_data *pdata = data->client->dev.platform_data;
	struct apds9930_coefficients defaults = {
		.ga	= APDS9930_DEF_GA,
		.coef_a	= APDS9930_COEF_SCALE,
		.coef_b	= APDS9930_DEF_B,
		.coef_c	= APDS9930_DEF_C,
		.coef_d	= APDS9930_DEF_D,
		.df	= APDS9930_DEF_DF,
	};
	struct device_node *np = data->client->dev.of_node;

	/* Look for platform data */
	if (pdata != NULL) {
		data->__again	= pdata->again;
		data->__atime	= pdata->atime;
		data->__pdrive	= pdata->pdrive;
		data->__ppulse	= pdata->ppulse;

		if (pdata->coefs.ga == 0)
			data->__coefs = defaults;
		else
			data->__coefs = pdata->coefs;

		return;
	}

	/* Look for device properties and set them to proper value or to
	 * default. */
	if (of_property_read_u32(np, APDS9930_GA_PROP, &data->__coefs.ga) != 0)
		data->__coefs.ga = defaults.ga;

	if (of_property_read_u32(np, APDS9930_DF_PROP, &data->__coefs.df) != 0)
		data->__coefs.df = defaults.df;

	if (of_property_read_u32(np, APDS9930_COEF_B_PROP,
				 &data->__coefs.coef_b) != 0 ||
	    of_property_read_u32(np, APDS9930_COEF_C_PROP,
				 &data->__coefs.coef_c) != 0 ||
	    of_property_read_u32(np, APDS9930_COEF_D_PROP,
				 &data->__coefs.coef_d) != 0) {
		data->__coefs.coef_b	= defaults.coef_b;
		data->__coefs.coef_c	= defaults.coef_c;
		data->__coefs.coef_d	= defaults.coef_d;
	}
	data->__coefs.coef_a = defaults.coef_a;

	if (of_property_read_u8(np, APDS9930_ATIME_PROP, &data->__atime) != 0)
		data->__atime = APDS9930_DEF_ATIME;

	if (of_property_read_u8(np, APDS9930_AGAIN_PROP, &data->__again) != 0)
		data->__again = APDS9930_DEF_AGAIN;

	/* We expect for the AGAIN value to be the one in the register (0, 1, 2
	 * or 3). If we do find device property AGAIN, but is not valid, fall
	 * back to the default one. */
	if (data->__again > APDS9930_MAX_AGAIN_INDEX)
		data->__again = APDS9930_DEF_AGAIN;

	if (of_property_read_u8(np, APDS9930_PDRIVE_PROP, &data->__pdrive) != 0)
		data->__pdrive = APDS9930_DEF_PDRIVE;

	if (of_property_read_u8(np, APDS9930_PPULSE_PROP, &data->__ppulse) != 0)
		data->__ppulse = APDS9930_DEF_PPULSE;
}

static void apds9930_chip_data_init(struct apds9930_data *data)
{
	apds9930_set_platform_data(data);

	/* Init data to default values */
	data->alsit		= apds9930_atime_to_alsit(data->__atime);
	data->ch0_max		= apds9930_compute_max_ch0(APDS9930_DEF_ATIME);
	data->agl_enabled	= false;
	data->als_intr_state	= false;
	data->ps_intr_state	= false;
	data->als_thresh.low	= APDS9930_DEF_ALS_THRESH_LOW;
	data->als_thresh.high	= APDS9930_DEF_ALS_THRESH_HIGH;
	data->ps_thresh.low	= APDS9930_DEF_PS_THRESH_LOW;
	data->ps_thresh.high	= APDS9930_DEF_PS_THRESH_HIGH;
}

/* Basic chip initialization, as described in the datasheet */
static int apds9930_chip_registers_init(struct apds9930_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	/* Disable and powerdown device */
	ret = apds9930_disable_all(data);
	if (ret < 0)
		return ret;

	/* Set timing registers default values (minimum) */
	ret = apds9930_write_byte(client, APDS9930_ATIME_REG, (data->__atime));
	if (ret < 0)
		return ret;

	ret = apds9930_write_byte(client, APDS9930_PTIME_REG,
				  APDS9930_DEF_PTIME);
	if (ret < 0)
		return ret;
	ret = apds9930_write_byte(client, APDS9930_WTIME_REG,
				  APDS9930_DEF_WTIME);
	if (ret < 0)
		return ret;

	/* Interrupt threshold default settings */
	ret = apds9930_write_word(client, APDS9930_AILTL_REG,
				  APDS9930_DEF_ALS_THRESH_LOW);
	if (ret < 0)
		return ret;
	ret = apds9930_write_word(client, APDS9930_AIHTL_REG,
				  APDS9930_DEF_ALS_THRESH_HIGH);
	if (ret < 0)
		return ret;
	ret = apds9930_write_word(client, APDS9930_PILTL_REG,
				  APDS9930_DEF_PS_THRESH_LOW);
	if (ret < 0)
		return ret;
	ret = apds9930_write_word(client, APDS9930_PIHTL_REG,
				  APDS9930_DEF_PS_THRESH_HIGH);
	if (ret < 0)
		return ret;

	/* Set persistance filters to default values */
	ret = apds9930_write_byte(client, APDS9930_PERS_REG,
				  APDS9930_DEF_APERS << APDS9930_APERS_SHIFT |
				  APDS9930_DEF_PPERS << APDS9930_PPERS_SHIFT);
	if (ret < 0)
		return ret;

	/* Reset the configuration register (do not wait long) */
	ret = apds9930_write_byte(client, APDS9930_CONFIG_REG, 0);
	if (ret < 0)
		return ret;

	/* Set proximity pulse count register (number of pulses to be generated
	 * during the proximity accum state)
	 */
	ret = apds9930_write_byte(client, APDS9930_PPULSE_REG, data->__ppulse);
	if (ret < 0)
		return ret;

	/* Gain selection setting */
	ret = apds9930_write_byte(client, APDS9930_CONTROL_REG,
				  data->__again << APDS9930_AGAIN_SHIFT |
				  APDS9930_DEF_PGAIN << APDS9930_PGAIN_SHIFT |
				  APDS9930_DEF_PDIODE << APDS9930_PDIODE_SHIFT |
				  data->__pdrive << APDS9930_PDRIVE_SHIFT);
	if (ret < 0)
		return ret;

	/* Power the device back on */
	return apds9930_enable_all(data);
}

/* Raw reading implementation */
static int apds9930_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct apds9930_data *data	= iio_priv(indio_dev);
	struct i2c_client *client	= data->client;
	int ret;
	u16 ch0, ch1, ch;

	mutex_lock(&data->mutex);
	switch (chan->type) {
	case IIO_LIGHT:
		/* Get Lux value */
		ret = apds9930_read_word(client, APDS9930_CDATAL_REG, &ch0);
		if (ret < 0)
			break;
		ret = apds9930_read_word(client, APDS9930_IRDATAL_REG, &ch1);
		if (ret < 0)
			break;

		/* Compute Lux value and check its validity */
		*val = apds9930_compute_lux(data, ch0, ch1);
		apds9930_update_again(data, ch0);
		ret = IIO_VAL_INT;
		break;
	case IIO_INTENSITY:
		/* Get ch0 or ch1 raw value */
		ret = apds9930_read_word(client, chan->channel ?
					 APDS9930_IRDATAL_REG :
					 APDS9930_CDATAL_REG,
					 &ch);
		if (ret < 0)
			break;

		*val	= (int)ch;
		ret	= IIO_VAL_INT;
		break;
	case IIO_PROXIMITY:
		/* Get proximity raw value */
		ret = apds9930_read_word(client, APDS9930_PDATAL_REG, &ch);
		if (ret < 0)
			break;

		*val	= APDS9930_MAX_PS_THRESH - (int)ch;
		ret	= IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&data->mutex);

	return ret;
}

/* Event handling functions */
static int apds9930_read_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir)
{
	struct apds9930_data *data = iio_priv(indio_dev);

	switch (chan->type) {
	case IIO_INTENSITY:
		return data->als_intr_state;
	case IIO_PROXIMITY:
		return data->ps_intr_state;
	default:
		return -EINVAL;
	}
}

static int apds9930_write_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       int state)
{
	struct apds9930_data *data	= iio_priv(indio_dev);
	struct i2c_client *client	= data->client;
	u8 enable_reg, which_intr;
	int ret;
	bool *intr_state_addr;

	if (chan->type != IIO_INTENSITY)
		return -EINVAL;

	mutex_lock(&data->mutex);

	ret = apds9930_read_byte(client, APDS9930_ENABLE_REG, &enable_reg);
	if (ret < 0)
		goto err;

	switch (chan->type) {
	case IIO_INTENSITY:
		which_intr	= APDS9930_AIEN;
		intr_state_addr	= &(data->als_intr_state);
		break;
	case IIO_PROXIMITY:
		which_intr	= APDS9930_PIEN;
		intr_state_addr	= &(data->ps_intr_state);
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	if (state)
		enable_reg |= which_intr;	/* enable */
	else
		enable_reg &= ~which_intr;	/* disable */

	ret = apds9930_write_byte(client, APDS9930_ENABLE_REG, enable_reg);
	if (ret == 0)
		*intr_state_addr = (bool)state;
err:
	mutex_unlock(&data->mutex);

	return ret;
}

static int apds9930_read_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int *val, int *val2)
{
	struct apds9930_data *data = iio_priv(indio_dev);
	struct apds9930_threshold thresh;

	switch (chan->type) {
	case IIO_INTENSITY:
		thresh = data->als_thresh;
		break;
	case IIO_PROXIMITY:
		thresh = data->ps_thresh;
		break;
	default:
		return -EINVAL;
	}

	switch (dir) {
	case IIO_EV_DIR_RISING:
		*val = (int)(thresh.high);
		break;
	case IIO_EV_DIR_FALLING:
		*val = (int)(thresh.low);
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

/* IIO device specific data structures */
static const struct iio_info apds9930_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= apds9930_read_raw,
	.read_event_config	= apds9930_read_event_config,
	.write_event_config	= apds9930_write_event_config,
	.read_event_value	= apds9930_read_event_value,
};

/* Event specs for ALS and PS thresholds. Both of them behave in the same
 * manner, thus define only one set of specifications.
 */
static const struct iio_event_spec apds9930_event_spec[] = {
	{
		/* "above threshold" event */
		.type		= IIO_EV_TYPE_THRESH,
		.dir		= IIO_EV_DIR_RISING,
		.mask_separate	= BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		/* "below threshold" event */
		.type		= IIO_EV_TYPE_THRESH,
		.dir		= IIO_EV_DIR_FALLING,
		.mask_separate	= BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec apds9930_channels[] = {
	{
		/* Lux (processed Ch0 + Ch1) */
		.type			= IIO_LIGHT,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_PROCESSED),
	}, {
		/* Ch0 photodiode (visible light + infrared); threshold
		 * triggered event */
		.type			= IIO_INTENSITY,
		.channel		= 0,
		.modified		= true,
		.channel2		= IIO_MOD_LIGHT_BOTH,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.event_spec		= apds9930_event_spec,
		.num_event_specs	= ARRAY_SIZE(apds9930_event_spec),
	}, {
		/* Ch1 photodiode (infrared) */
		.type			= IIO_INTENSITY,
		.channel		= 1,
		.modified		= true,
		.channel2		= IIO_MOD_LIGHT_IR,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
	}, {
		/* Proximity channel; threshold triggered event */
		.type			= IIO_PROXIMITY,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),
		.event_spec		= apds9930_event_spec,
		.num_event_specs	= ARRAY_SIZE(apds9930_event_spec),
	}
};

/* Determine what has triggered the interrupt and clear it accordingly */
static int apds9930_interrupt_clear(struct apds9930_data *data, u8 intr_status)
{
	u8 reg = 0x00;

	APDS9930_CMD_REG_SETUP(reg, APDS9930_CMD_TYPE_SPECIAL_FUNC);

	switch (intr_status & (APDS9930_AINT | APDS9930_PINT)) {
	case APDS9930_AINT:
		reg |= APDS9930_CMD_TYPE_ALS;
		break;
	case APDS9930_PINT:
		reg |= APDS9930_CMD_TYPE_PS;
		break;
	case (APDS9930_AINT & APDS9930_PINT):
		reg |= APDS9930_CMD_TYPE_BOTH;
		break;
	default:
		return -EINVAL;
	}

	return i2c_smbus_read_byte_data(data->client, reg);
}

/* ALS interrupt handler */
static irqreturn_t apds9930_irq_handler(int irq, void *private_data)
{
	struct iio_dev *indio_dev	= private_data;
	struct apds9930_data *data	= iio_priv(indio_dev);
	u8 status, enable_reg;
	u16 ch0, ps_data;
	int ret;

	/* Disable ADCs converters while processing data */
	ret = apds9930_read_byte(data->client, APDS9930_ENABLE_REG,
				 &enable_reg);
	if (ret < 0)
		return IRQ_HANDLED;
	ret = apds9930_write_byte(data->client, APDS9930_ENABLE_REG, 1);
	if (ret < 0)
		return IRQ_HANDLED;

	/* Read status register to see what caused the interrupt */
	ret = apds9930_read_byte(data->client, APDS9930_STATUS_REG, &status);
	if (ret < 0)
		goto err;

	/* Push event to userspace */
	if (status & APDS9930_AINT) {
		/* Clear interrupt */
		apds9930_interrupt_clear(data, status);

		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_INTENSITY,
						  0,
						  IIO_MOD_LIGHT_BOTH,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_EITHER),
			       iio_get_time_ns());

		ret = apds9930_read_word(data->client, APDS9930_CDATAL_REG,
					 &ch0);
		if (ret < 0)
			goto err;

		apds9930_update_again(data, ch0);

		/* Update ALS thresholds to environment */
		apds9930_update_als_thresholds(data, ch0);
	}

	if (status & APDS9930_PINT) {
		/* Clear interrupt */
		apds9930_interrupt_clear(data, status);

		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
						    0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns());

		ret = apds9930_read_word(data->client, APDS9930_PDATAL_REG,
					 &ps_data);
		if (ret < 0)
			goto err;

		apds9930_update_ps_thresholds(data, ps_data);
	}

err:
	/* Re-enable converters */
	apds9930_write_byte(data->client, APDS9930_ENABLE_REG, enable_reg);

	return IRQ_HANDLED;
}

static int apds9930_gpio_probe(struct i2c_client *client,
			       struct apds9930_data *data)
{
	struct device *dev;
	struct gpio_desc *gpio;
	int ret;

	if (!client)
		return -EINVAL;

	dev = &client->dev;

	gpio = devm_gpiod_get_index(dev, APDS9930_GPIO_NAME, 0);
	if (IS_ERR(gpio)) {
		dev_err(dev, "ACPI GPIO get index failed\n");
		return PTR_ERR(gpio);
	}

	ret = gpiod_direction_input(gpio);
	if (ret)
		return ret;

	return gpiod_to_irq(gpio);
}

static int apds9930_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct apds9930_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	mutex_init(&data->mutex);

	/* Recommended wait time after power on. */
	msleep(APDS9930_INIT_SLEEP);

	/* Check if the chip is the one we are expecting */
	ret = apds9930_chip_detect(data);
	if (ret < 0)
		goto err;

	apds9930_chip_data_init(data);
	ret = apds9930_chip_registers_init(data);
	if (ret < 0)
		goto err;

	indio_dev->dev.parent	= &client->dev;
	indio_dev->name		= APDS9930_DRIVER_NAME;
	indio_dev->modes	= INDIO_DIRECT_MODE;
	indio_dev->channels	= apds9930_channels;
	indio_dev->num_channels	= ARRAY_SIZE(apds9930_channels);
	indio_dev->info		= &apds9930_info;

	if (client->irq <= 0)
		client->irq = apds9930_gpio_probe(client, data);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev,
						client->irq,
						NULL,
						apds9930_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						APDS9930_IRQ_NAME,
						indio_dev);
		if (ret < 0)
			return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto err;

	return 0;
err:
	apds9930_disable_all(data);

	return ret;
}

static int apds9930_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev	= i2c_get_clientdata(client);
	struct apds9930_data *data	= iio_priv(indio_dev);
	int ret;

	ret = apds9930_disable_all(data);
	iio_device_unregister(indio_dev);

	return ret;
}

static int apds9930_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct apds9930_data *data = iio_priv(indio_dev);
	int ret = 0;

        dev_dbg(dev, "%s: suspend\n", APDS9930_DRIVER_NAME);
	if(data->client->irq > 0) {
		disable_irq_nosync(data->client->irq);
	}

	ret = apds9930_disable_all(data);

        return ret;
}

static int apds9930_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct apds9930_data *data = iio_priv(indio_dev);
        int ret = 0;

        dev_dbg(dev, "%s: resume\n", APDS9930_DRIVER_NAME);

	ret = apds9930_enable_all(data);
	if(data->client->irq > 0) {
		enable_irq(data->client->irq);
	}

        return ret;
}

static const struct acpi_device_id apds9930_acpi_table[] = {
	{"APDS9930", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, apds9930_acpi_table);

static const struct i2c_device_id apds9930_ids_table[] = {
	{"apds9930", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, apds9930_ids_table);

static const struct dev_pm_ops apds9930_pm = {
        SET_SYSTEM_SLEEP_PM_OPS(apds9930_suspend, apds9930_resume)
};

static struct i2c_driver apds9930_iio_driver = {
	.driver	= {
		.name			= APDS9930_DRIVER_NAME,
		.acpi_match_table	= ACPI_PTR(apds9930_acpi_table),
		.owner			= THIS_MODULE,
		.pm			= &apds9930_pm,
	},
	.probe		= apds9930_probe,
	.remove		= apds9930_remove,
	.id_table	= apds9930_ids_table,
};
module_i2c_driver(apds9930_iio_driver);

MODULE_AUTHOR("Cristina Ciocan <cristina.ciocan@intel.com>");
MODULE_DESCRIPTION("APDS-9930 ALS sensor IIO driver");
MODULE_LICENSE("GPL");
