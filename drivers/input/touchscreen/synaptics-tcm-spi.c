// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TCM1 SPI touchscreen driver (minimal mainline port)
 *
 * Handles the Synaptics TouchCom v1 protocol over SPI for the S3910 and
 * compatible controllers.  Firmware loading is skipped -- the bootloader
 * already puts the chip in application mode and the driver just needs to
 * enable reports and parse touch data.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/firmware.h>

/*
 * Reflashing the touch controller's own flash is destructive and is never
 * needed on retail hardware (the vendor firmware is already programmed and
 * Android restores it if it is ever lost).  A single flaky IDENTIFY read
 * used to be enough to kick off an erase/write that could hang the boot and
 * leave the controller blank, so it is now strictly opt-in.
 */
static bool flash_firmware;
module_param(flash_firmware, bool, 0644);
MODULE_PARM_DESC(flash_firmware,
	"Reflash the touch controller firmware when it is not in application mode (dangerous; default off)");

/*
 * Diagnostic raw-frame capture.  Write N to dump the next N distinct touch
 * frames to the kernel log; 0 disables it.  Used by the touch-diag userspace
 * service to decode the on-wire object layout without a serial console.
 */
static unsigned int dump_frames;
static unsigned int dump_count;
static u32 dump_hash;
static u16 dump_len;

static int set_dump_frames(const char *val, const struct kernel_param *kp)
{
	int ret = kstrtouint(val, 0, &dump_frames);

	if (ret)
		return ret;
	dump_count = 0;
	return 0;
}

static const struct kernel_param_ops dump_frames_ops = {
	.set = set_dump_frames,
	.get = param_get_uint,
};
module_param_cb(dump_frames, &dump_frames_ops, &dump_frames, 0644);
MODULE_PARM_DESC(dump_frames,
	"Dump the next N distinct raw touch frames to the kernel log (default 0)");

/* ---- TCM v1 protocol constants ---- */

#define TCM_HEADER_SIZE			4
#define TCM_MSG_MAX			4096

/* TCM v1 marker */
#define TCM_V1_MARKER			0xa5
#define TCM_V1_PADDING			0x5a

/*
 * TCM v1 header (4 bytes):
 *   byte 0: marker (0xa5)
 *   byte 1: code (command/status/report)
 *   byte 2: length[7:0]   (payload length, LE)
 *   byte 3: length[15:8]
 *
 * Response: [0xa5] [code] [len_lo] [len_hi] [payload...] [0x5a padding]
 * Command:  [cmd] [len_lo] [len_hi] [payload...]
 */

/* Command codes */
#define TCM_CMD_SLEEP_OUT		0x01
#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_RESET			0x04
#define TCM_CMD_ENABLE_REPORT		0x05
#define TCM_CMD_DISABLE_REPORT		0x06
#define TCM_CMD_GET_BOOT_INFO		0x10
#define TCM_CMD_ERASE_FLASH		0x11
#define TCM_CMD_WRITE_FLASH		0x12
#define TCM_CMD_RUN_APP_FW		0x14
#define TCM_CMD_GET_APP_INFO		0x20
#define TCM_CMD_GET_DYNAMIC_CONFIG	0x23
#define TCM_CMD_SET_DYNAMIC_CONFIG	0x24
#define TCM_CMD_GET_TOUCH_REPORT_CONFIG	0x25
#define TCM_CMD_SET_TOUCH_REPORT_CONFIG	0x26
#define TCM_CMD_EXIT_DEEP_SLEEP		0x2d
#define TCM_CMD_RUN_BL_FW		0x1f

/* Report codes */
#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH		0x11
#define TCM_REPORT_HBP_ACTIVE_FRAME	0x23

/* Status codes */
#define STATUS_IDLE			0x00
#define STATUS_OK			0x01
#define STATUS_ERROR			0x0f

/* Firmware modes */
#define MODE_APPLICATION		0x01
#define MODE_APPLICATION_03		0x03	/* S3910 uses 0x03 */
#define MODE_BOOTLOADER			0x0b

/* Firmware image format */
#define FW_IMG_MAGIC			0x4818472b
#define FW_AREA_MAGIC			0x7c05e516
#define FW_AREA_HDR_SIZE		36
#define FW_UP_OFFSET			0x50

/* Touch object classifications */
#define TCM_OBJ_FINGER			0x03
#define TCM_OBJ_LARGEST_OBJECT		0x01

/* Touch report config descriptor codes */
#define TOUCH_END			0
#define TOUCH_FOREACH_ACTIVE_OBJECT	1
#define TOUCH_FOREACH_OBJECT		2
#define TOUCH_FOREACH_END		3
#define TOUCH_PAD_TO_NEXT_BYTE		4
#define TOUCH_TIMESTAMP			5
#define TOUCH_OBJECT_N_INDEX		6
#define TOUCH_OBJECT_N_CLASSIFICATION	7
#define TOUCH_OBJECT_N_X_POSITION	8
#define TOUCH_OBJECT_N_Y_POSITION	9
#define TOUCH_OBJECT_N_Z		10
#define TOUCH_OBJECT_N_X_WIDTH		11
#define TOUCH_OBJECT_N_Y_WIDTH		12
#define TOUCH_OBJECT_N_TX_TIXELS	13
#define TOUCH_OBJECT_N_RX_TIXELS	14
#define TOUCH_0D_BUTTONS_STATE		15
#define TOUCH_GESTURE_DOUBLE_TAP	16
#define TOUCH_FRAME_RATE		17
#define TOUCH_NUM_OF_ACTIVE_OBJECTS	24
#define TOUCH_REPORT_GESTURE_INFO	198
#define TOUCH_REPORT_GESTURE_COORDINATE 199
#define TOUCH_REPORT_CUSTOMER_GRIP_INFO 203

/* Unknown codes in switch default - just skip their bit_width */

/* Response timeout */
#define TCM_CMD_TIMEOUT_MS		3000
#define TCM_MAX_SLOTS			10

/* Detect magic byte sent raw (not wrapped in TCM message) */
#define TCM_DETECT_MAGIC		0x07

/* ---- Debug hex dump ---- */

static void tcm_hexdump(const char *tag, const u8 *data, unsigned int len)
{
	char buf[256];
	unsigned int i, pos = 0;

	for (i = 0; i < len && pos + 4 < sizeof(buf); i++)
		pos += scnprintf(buf + pos, sizeof(buf) - pos, "%02x ", data[i]);
	pr_info("syna-tcm: %s[%u]: %s\n", tag, len, buf);
}

/* ---- Device context ---- */

struct tcm_hw {
	struct spi_device *spi;
	struct input_dev *input;
	struct regulator *avdd;
	struct gpio_desc *reset;
	struct gpio_desc *irq;
	int irq_num;
	u8 *buf;		/* shared read buffer (TCM_MSG_MAX), allocated in probe */
	u8 *irq_buf;		/* separate buffer for IRQ handler */

	u8 firmware_mode;
	u16 max_write_size;	/* from identify, e.g. 1024 */
	u16 fw_max_x;
	u16 fw_max_y;
	u16 panel_max_x;
	u16 panel_max_y;
	u8 max_objects;

	/*
	 * Optional edge calibration: the sensor's usable area does not quite
	 * reach the glass edges, so touches near the border read short.  When
	 * set, [cal_min, cal_max] (in panel pixels) is stretched to the full
	 * [0, panel_max].  Zero means disabled.
	 */
	u16 cal_x_min;
	u16 cal_x_max;
	u16 cal_y_min;
	u16 cal_y_max;

	/* Touch report config (config-driven parser) */
	u8 *touch_config;
	u16 touch_config_size;
	u16 max_touch_report_config_size;
	u16 max_touch_report_payload_size;
	bool use_default_format;	/* firmware default 32+8N format */

};

/* ---- Low-level SPI transfer ---- */

/*
 * Raw SPI read — reads exactly len bytes using full-duplex mode.
 */
static int tcm_spi_read_raw(struct spi_device *spi, u8 *buf, unsigned int len)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	u8 *tx;
	int ret;

	tx = kmalloc(len, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;
	memset(tx, 0xff, len);

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = tx;
	xfer.rx_buf = buf;
	xfer.len = len;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(spi, &msg);
	kfree(tx);
	return ret;
}

/*
 * SPI read with prefix handling.
 *
 * The mt65xx SPI controller on MT6991 always prepends a stale 0x25 byte
 * to every read (even in full-duplex mode).  Work around this by always
 * reading len+1 bytes and discarding byte 0.
 */
static int tcm_spi_read(struct tcm_hw *hw, u8 *buf, unsigned int len)
{
	u8 *tmp;
	int ret;

	if (len + 1 > TCM_MSG_MAX + 1)
		return -EINVAL;

	tmp = kmalloc(len + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = tcm_spi_read_raw(hw->spi, tmp, len + 1);
	if (ret == 0)
		memcpy(buf, tmp + 1, len);
	kfree(tmp);
	return ret;
}

static int tcm_spi_write(struct tcm_hw *hw, const u8 *buf, unsigned int len)
{
	return spi_write(hw->spi, buf, len);
}

/* ---- TCM v1 command/response ---- */

/*
 * Build and send a TCM v1 command packet:
 *   [code] [len_lo] [len_hi] [payload...]
 *
 * No CRC6 in header, no marker prefix.
 */
static int tcm_write_cmd(struct tcm_hw *hw, u8 cmd,
			  const u8 *payload, u16 payload_len)
{
	u8 *msg;
	int total;
	int ret;

	total = 3 + payload_len;
	msg = kzalloc(total, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg[0] = cmd;
	msg[1] = payload_len & 0xff;
	msg[2] = (payload_len >> 8) & 0xff;
	if (payload_len > 0 && payload)
		memcpy(msg + 3, payload, payload_len);

	ret = tcm_spi_write(hw, msg, total);
	kfree(msg);
	return ret;
}

/*
 * Read a TCM v1 response.
 *
 * Format: [0xa5] [code] [len_lo] [len_hi] [payload...] [0x5a padding]
 *
 * The mt65xx SPI controller always puts a stale 0x25 byte at position 0
 * of each SPI read.  tcm_spi_read() strips it.  When the half-duplex
 * spi_write() clocks in the IC's first response byte via MISO (and
 * discards it), the 0xa5 marker is lost and the code byte appears at
 * buf[0].  We detect this and reconstruct the header.
 *
 * Returns the number of extra payload bytes saved (0 or 2).
 */
static int tcm_read_response(struct tcm_hw *hw, u8 *buf, unsigned int bufsize)
{
	u16 payload_len;
	int ret, retry;
	u8 saved[2];
	int saved_cnt = 0;

	/* Wait for ATTN to go low */
	{
		unsigned long timeout = jiffies + msecs_to_jiffies(TCM_CMD_TIMEOUT_MS);

		while (time_before(jiffies, timeout)) {
			if (!gpiod_get_value(hw->irq))
				break;
			usleep_range(500, 1000);
		}
		if (gpiod_get_value(hw->irq))
			return -ETIMEDOUT;
	}

	/*
	 * Read TCM_HEADER_SIZE + 1 bytes.  After the 0x25 prefix strip,
	 * buf holds IC bytes from the start of the response:
	 *
	 *  a5 survived:  [a5] [code] [len_lo] [len_hi] [payload0]
	 *  a5 consumed:  [code] [len_lo] [len_hi] [payload0] [payload1]
	 */
	for (retry = 0; retry < 10; retry++) {
		ret = tcm_spi_read(hw, buf, TCM_HEADER_SIZE + 1);
		if (ret < 0)
			return ret;

		/* Case 1: 0xa5 marker survived. */
		if (buf[0] == TCM_V1_MARKER) {
			saved[0] = buf[TCM_HEADER_SIZE]; /* payload0 */
			saved_cnt = 1;
			break;
		}

		/*
		 * Case 2: marker consumed by trigger/command write.
		 * buf[0] is the response code; length at buf[1..2] is LE.
		 * Any non-marker value that could be a TCM response code
		 * (0x00..0x7f) triggers header reconstruction.
		 */
		if (buf[0] != TCM_V1_MARKER && buf[0] < 0x80) {
			saved[0] = buf[3]; /* payload0 */
			saved[1] = buf[4]; /* payload1 */
			saved_cnt = 2;
			memmove(buf + 1, buf, 3);
			buf[0] = TCM_V1_MARKER;
			pr_debug("syna-tcm: marker consumed, code=0x%02x\n",
				 buf[1]);
			break;
		}

		pr_warn("syna-tcm: bad marker 0x%02x %02x %02x %02x %02x (retry %d)\n",
			buf[0], buf[1], buf[2], buf[3], buf[4], retry);
		usleep_range(1000, 2000);
	}

	if (buf[0] != TCM_V1_MARKER)
		return -EIO;

	/* TCM v1: code at byte 1, length at bytes 2-3 (LE) */
	payload_len = buf[2] | (buf[3] << 8);

	if (payload_len == 0)
		return 0;

	if (TCM_HEADER_SIZE + payload_len + 10 > bufsize)
		return -EINVAL;

	/* Prepend saved payload bytes, then read the rest. */
	if (saved_cnt) {
		memcpy(buf + TCM_HEADER_SIZE, saved, saved_cnt);
		ret = tcm_spi_read(hw, buf + TCM_HEADER_SIZE + saved_cnt,
				   payload_len + 10 - saved_cnt);
	} else {
		ret = tcm_spi_read(hw, buf + TCM_HEADER_SIZE,
				   payload_len + 10);
	}

	return ret;
}

static int tcm_send_cmd(struct tcm_hw *hw, u8 cmd,
			const u8 *payload, u16 len)
{
	return tcm_write_cmd(hw, cmd, payload, len);
}

/* Forward declarations */
static void tcm_parse_touch(struct tcm_hw *hw, const u8 *data, u16 len);

/*
 * Read a command response, skipping any async touch reports (0x11)
 * that the IC may be sending between command responses.
 * Touch reports are dispatched to tcm_parse_touch inline.
 */
static int tcm_read_cmd_response(struct tcm_hw *hw, u8 *buf,
				 unsigned int bufsize)
{
	int ret;
	unsigned long timeout;
	int attempts = 0;

	timeout = jiffies + msecs_to_jiffies(TCM_CMD_TIMEOUT_MS);
	while (time_before(jiffies, timeout) && attempts < 50) {
		ret = tcm_read_response(hw, buf, bufsize);
		if (ret < 0) {
			attempts++;
			usleep_range(1000, 2000);
			continue;
		}

		/* Touch report: dispatch and keep reading */
		if (buf[1] == TCM_REPORT_TOUCH) {
			u16 plen = buf[2] | (buf[3] << 8);

			if (plen > 0 && TCM_HEADER_SIZE + plen <= bufsize)
				tcm_parse_touch(hw,
					buf + TCM_HEADER_SIZE, plen);
			attempts++;
			continue;
		}

		/* Not a touch report — this is our command response */
		return 0;
	}

	return -ETIMEDOUT;
}

/* ---- TCM v1 protocol helpers ---- */

/*
 * Parse identify info from a REPORT_IDENTIFY response buffer.
 *
 * Empirical layout (matches vendor build id / max write chunk):
 *   [0]     version
 *   [1]     mode
 *   [2]     generation / proto revision
 *   [3-18]  part_number (16 bytes)
 *   [19-22] build_id (LE32)
 *   [23-24] max_write_size (LE16)
 *   [25-26] max_read_size (LE16)
 */
static void tcm_parse_identify(struct tcm_hw *hw, const u8 *buf)
{
	u16 payload_len = buf[2] | (buf[3] << 8);
	const u8 *p = buf + TCM_HEADER_SIZE;

	if (payload_len >= 27) {
		hw->firmware_mode = p[1];
		hw->max_write_size = p[23] | (p[24] << 8);
		pr_info("syna-tcm: ver=%u mode=0x%02x gen=%u\n",
			p[0], p[1], p[2]);
		tcm_hexdump("part", &p[3], 16);
		pr_info("syna-tcm: build=%02x%02x%02x%02x max_wr=%u max_rd=%u\n",
			p[19], p[20], p[21], p[22],
			p[23] | (p[24] << 8),
			p[25] | (p[26] << 8));
	} else if (payload_len >= 2) {
		hw->firmware_mode = p[1];
		pr_info("syna-tcm: mode=0x%02x (short identify, %u bytes)\n",
			p[1], payload_len);
		tcm_hexdump("id_raw", p, min_t(u16, payload_len, 32));
	} else {
		pr_info("syna-tcm: identify payload %u bytes\n", payload_len);
		tcm_hexdump("id_raw", p, min_t(u16, payload_len, 32));
	}
}

static int tcm_identify(struct tcm_hw *hw)
{
	int ret;
	unsigned long timeout;
	int attempts = 0;

	ret = tcm_send_cmd(hw, TCM_CMD_IDENTIFY, NULL, 0);
	if (ret < 0)
		return ret;
	pr_info("syna-tcm: identify cmd sent, ATTN=%d\n",
		gpiod_get_value(hw->irq));

	timeout = jiffies + msecs_to_jiffies(TCM_CMD_TIMEOUT_MS);
	while (time_before(jiffies, timeout) && attempts < 50) {
		ret = tcm_read_response(hw, hw->buf, TCM_MSG_MAX);
		if (ret < 0) {
			attempts++;
			usleep_range(1000, 2000);
			continue;
		}

		/* Skip async touch reports */
		if (hw->buf[1] == TCM_REPORT_TOUCH) {
			u16 plen = hw->buf[2] | (hw->buf[3] << 8);

			if (plen > 0 && TCM_HEADER_SIZE + plen <= TCM_MSG_MAX)
				tcm_parse_touch(hw,
					hw->buf + TCM_HEADER_SIZE, plen);
			attempts++;
			continue;
		}

		pr_info("syna-tcm: identify resp code=0x%02x len=%u\n",
			hw->buf[1], hw->buf[2] | (hw->buf[3] << 8));
		tcm_hexdump("identify", hw->buf,
			     min_t(unsigned int,
				   TCM_HEADER_SIZE +
				   (hw->buf[2] | (hw->buf[3] << 8)) + 10, 64));
		if (hw->buf[1] == TCM_REPORT_IDENTIFY)
			break;
		if (hw->buf[1] == STATUS_OK) {
			pr_info("syna-tcm: STATUS_OK for identify (len=%u)\n",
				hw->buf[2] | (hw->buf[3] << 8));
			tcm_parse_identify(hw, hw->buf);
			return 0;
		}
		if (hw->buf[1] == STATUS_ERROR) {
			pr_info("syna-tcm: STATUS_ERROR (already identified?)\n");
			tcm_parse_identify(hw, hw->buf);
			return 0;
		}
		attempts++;
		usleep_range(1000, 2000);
	}

	if (hw->buf[1] == TCM_REPORT_IDENTIFY)
		tcm_parse_identify(hw, hw->buf);

	return 0;
}

static int tcm_get_app_info(struct tcm_hw *hw)
{
	u8 *buf = hw->buf;
	int ret;

	ret = tcm_send_cmd(hw, TCM_CMD_GET_APP_INFO, NULL, 0);
	if (ret < 0)
		return ret;

	ret = tcm_read_cmd_response(hw, buf, TCM_MSG_MAX);
	if (ret < 0) {
		pr_err("syna-tcm: get_app_info failed: %d\n", ret);
		return ret;
	}

	pr_info("syna-tcm: app_info resp code=0x%02x len=%u\n",
		buf[1], buf[2] | (buf[3] << 8));
	tcm_hexdump("appinfo", buf + TCM_HEADER_SIZE,
		     min_t(unsigned int, buf[2] | (buf[3] << 8), 48));

	{
		u16 payload_len = buf[2] | (buf[3] << 8);
		u8 *p = buf + TCM_HEADER_SIZE;

		/*
		 * Vendor tcm_application_info layout (LE16 fields):
		 *   [0-1]   version
		 *   [2-3]   status
		 *   [4-5]   static_config_size
		 *   [6-7]   dynamic_config_size
		 *   [8-9]   app_config_start_write_block
		 *   [10-11] app_config_size
		 *   [12-13] max_touch_report_config_size
		 *   [14-15] max_touch_report_payload_size
		 *   [16-32] customer_config_id (length-prefixed, 17 bytes)
		 *   [33-34] max_x
		 *   [35-36] max_y
		 *   [37-38] max_objects
		 */
		if (payload_len >= 39) {
			hw->max_touch_report_config_size =
				p[12] | (p[13] << 8);
			hw->max_touch_report_payload_size =
				p[14] | (p[15] << 8);
			hw->fw_max_x = p[33] | (p[34] << 8);
			hw->fw_max_y = p[35] | (p[36] << 8);
			hw->max_objects = p[37] | (p[38] << 8);
			if (hw->max_objects > TCM_MAX_SLOTS)
				hw->max_objects = TCM_MAX_SLOTS;

			if (!hw->panel_max_x) {
				hw->panel_max_x = hw->fw_max_x;
				hw->panel_max_y = hw->fw_max_y;
			}

			pr_info("syna-tcm: fw_max=%dx%d panel=%dx%d obj=%d rpt_cfg=%u rpt_pl=%u\n",
				hw->fw_max_x, hw->fw_max_y,
				hw->panel_max_x, hw->panel_max_y,
				hw->max_objects,
				hw->max_touch_report_config_size,
				hw->max_touch_report_payload_size);
		} else {
			pr_warn("syna-tcm: app_info too short (%u bytes)\n",
				payload_len);
		}
	}

	return 0;
}

/* ---- Touch report config ---- */

/*
 * Read the touch report config currently active in the IC.
 *
 * The report layout is entirely described by this config: for each
 * field it gives an entity code and a bit width.  The parser walks the
 * config and extracts fields from the report bitstream, so we must have
 * the config before we can decode a single touch report.
 */
static int tcm_get_touch_report_config(struct tcm_hw *hw)
{
	int ret;
	u16 len;

	ret = tcm_send_cmd(hw, TCM_CMD_GET_TOUCH_REPORT_CONFIG, NULL, 0);
	if (ret < 0)
		return ret;

	ret = tcm_read_cmd_response(hw, hw->buf, TCM_MSG_MAX);
	if (ret < 0) {
		pr_warn("syna-tcm: GET_TOUCH_REPORT_CONFIG no response\n");
		return ret;
	}

	if (hw->buf[1] != STATUS_OK) {
		pr_warn("syna-tcm: GET_TOUCH_REPORT_CONFIG code=0x%02x\n",
			hw->buf[1]);
		return -EIO;
	}

	len = hw->buf[2] | (hw->buf[3] << 8);
	if (len == 0 || len > TCM_MSG_MAX - TCM_HEADER_SIZE)
		return -EINVAL;

	kfree(hw->touch_config);
	/* +2 zero padding so the parser can always read a bit width */
	hw->touch_config = kzalloc(len + 2, GFP_KERNEL);
	if (!hw->touch_config)
		return -ENOMEM;

	memcpy(hw->touch_config, hw->buf + TCM_HEADER_SIZE, len);
	hw->touch_config_size = len;

	pr_info("syna-tcm: touch report config %u bytes\n", len);
	tcm_hexdump("rpt_cfg", hw->touch_config, min_t(u16, len, 64));

	return 0;
}

static int tcm_set_touch_report_config(struct tcm_hw *hw)
{
	int ret;
	u16 cfg_size = hw->max_touch_report_config_size;
	u8 *payload;

	if (cfg_size == 0) {
		pr_warn("syna-tcm: no touch report config size\n");
		return 0;
	}

	/*
	 * Normal mode config descriptor from vendor:
	 *   GESTURE_DOUBLE_TAP (0x10)  8 bits
	 *   FOREACH_ACTIVE_OBJECT (0x01)
	 *     OBJECT_N_INDEX (0x06)       4 bits
	 *     OBJECT_N_CLASSIFICATION (0x07) 4 bits
	 *     OBJECT_N_X_POSITION (0x08)  16 bits
	 *     OBJECT_N_Y_POSITION (0x09)  16 bits
	 *     OBJECT_N_X_WIDTH (0x0B)     12 bits
	 *     OBJECT_N_Y_WIDTH (0x0C)     12 bits
	 *   FOREACH_END (0x03)
	 *   END (0x00)
	 */
	u8 cfg_desc[] = {
		TOUCH_GESTURE_DOUBLE_TAP, 8,
		TOUCH_FOREACH_ACTIVE_OBJECT,
		TOUCH_OBJECT_N_INDEX, 4,
		TOUCH_OBJECT_N_CLASSIFICATION, 4,
		TOUCH_OBJECT_N_X_POSITION, 16,
		TOUCH_OBJECT_N_Y_POSITION, 16,
		TOUCH_OBJECT_N_X_WIDTH, 12,
		TOUCH_OBJECT_N_Y_WIDTH, 12,
		TOUCH_FOREACH_END,
		TOUCH_END,
	};
	unsigned int desc_size = sizeof(cfg_desc);

	/*
	 * Vendor sends the full max_touch_report_config_size bytes.
	 * TCM v1 header length field is 16-bit, supports up to 65535.
	 */
	payload = kzalloc(cfg_size, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;
	memcpy(payload, cfg_desc, min(desc_size, (unsigned int)cfg_size));

	pr_info("syna-tcm: setting touch report config (%u bytes)\n", cfg_size);
	tcm_hexdump("rpt_cfg", payload, min_t(unsigned int, desc_size, 32));

	ret = tcm_send_cmd(hw, TCM_CMD_SET_TOUCH_REPORT_CONFIG,
			   payload, cfg_size);
	kfree(payload);
	if (ret < 0) {
		pr_err("syna-tcm: set_touch_report_config failed: %d\n", ret);
		return ret;
	}

	/* Read response */
	ret = tcm_read_cmd_response(hw, hw->buf, TCM_MSG_MAX);

	if (ret == 0) {
		pr_info("syna-tcm: touch report config set, code=0x%02x\n",
			hw->buf[1]);
		/* Save the config descriptor for the parser */
		kfree(hw->touch_config);
		hw->touch_config = kzalloc(desc_size, GFP_KERNEL);
		if (hw->touch_config) {
			unsigned int idx = 0;

			hw->touch_config[idx++] = TOUCH_GESTURE_DOUBLE_TAP;
			hw->touch_config[idx++] = 8;
			hw->touch_config[idx++] = TOUCH_FOREACH_ACTIVE_OBJECT;
			hw->touch_config[idx++] = TOUCH_OBJECT_N_INDEX;
			hw->touch_config[idx++] = 4;
			hw->touch_config[idx++] = TOUCH_OBJECT_N_CLASSIFICATION;
			hw->touch_config[idx++] = 4;
			hw->touch_config[idx++] = TOUCH_OBJECT_N_X_POSITION;
			hw->touch_config[idx++] = 16;
			hw->touch_config[idx++] = TOUCH_OBJECT_N_Y_POSITION;
			hw->touch_config[idx++] = 16;
			hw->touch_config[idx++] = TOUCH_OBJECT_N_X_WIDTH;
			hw->touch_config[idx++] = 12;
			hw->touch_config[idx++] = TOUCH_OBJECT_N_Y_WIDTH;
			hw->touch_config[idx++] = 12;
			hw->touch_config[idx++] = TOUCH_FOREACH_END;
			hw->touch_config[idx++] = TOUCH_END;
			hw->touch_config_size = idx;
		}
	} else {
		pr_err("syna-tcm: set_touch_report_config response timeout\n");
	}

	return ret;
}

/* ---- Touch report parsing (config-driven) ---- */

static unsigned int tcm_get_bits(const u8 *buf, unsigned int bit_offset,
				 unsigned int num_bits)
{
	unsigned int byte_offset = bit_offset / 8;
	unsigned int bit_in_byte = bit_offset % 8;
	unsigned int result = 0;
	unsigned int remaining = num_bits;

	while (remaining > 0 && byte_offset < TCM_MSG_MAX) {
		unsigned int avail = 8 - bit_in_byte;
		unsigned int take = (remaining < avail) ? remaining : avail;
		unsigned int mask = (1 << take) - 1;

		result |= ((buf[byte_offset] >> bit_in_byte) & mask) <<
			  (num_bits - remaining);
		bit_in_byte = 0;
		byte_offset++;
		remaining -= take;
	}
	return result;
}

static void tcm_parse_touch(struct tcm_hw *hw, const u8 *data, u16 len)
{
	int obj;
	u16 obj_x[TCM_MAX_SLOTS];
	u16 obj_y[TCM_MAX_SLOTS];
	u8 obj_status[TCM_MAX_SLOTS];
	u8 obj_width[TCM_MAX_SLOTS];
	unsigned int active_objects = 0;

	if (!hw->input)
		return;

	memset(obj_x, 0, sizeof(obj_x));
	memset(obj_y, 0, sizeof(obj_y));
	memset(obj_status, 0, sizeof(obj_status));
	memset(obj_width, 0, sizeof(obj_width));

	if (dump_frames && dump_count < dump_frames) {
		u32 h = 2166136261u;
		unsigned int i, n = min_t(u16, len, 96);

		for (i = 0; i < n; i++) {
			h ^= data[i];
			h *= 16777619u;
		}
		if (h != dump_hash || len != dump_len) {
			dump_hash = h;
			dump_len = len;
			pr_info("syna-tcm: rpt[%u] len=%u\n", dump_count, len);
			tcm_hexdump("raw_rpt", data, min_t(u16, len, 64));
			dump_count++;
		}
	}

	if (hw->use_default_format) {
		/*
		 * Observed S3910 default report format:
		 *   32-byte prefix + N x 12-byte objects.
		 * Object: [??] [0x10|finger] [x_lo] [x_hi] [y_lo] [y_hi]
		 *         [w0] [w1] [grip x4]
		 *
		 * byte 1 carries the finger number in its low nibble
		 * (0x10 = first finger, 0x11 = second, ...).  byte 0 is
		 * not a usable index: a two-finger report has 0x02/0x00
		 * there, which used to drop the second finger.
		 */
		unsigned int obj_size = 12;
		unsigned int base = 32;
		unsigned int off, i;

		if (len < 32)
			goto done;

		/*
		 * The byte-stream framing occasionally slips by one byte
		 * (observed prefixes "00 00 03 00" / "00 00 a5 03" /
		 * "00 03 00 00"), which would shift every object and make the
		 * old fixed offset 32 read garbage such as a bogus finger
		 * 0x01/0x80 with width ~106-128.  Relocate the object array by
		 * finding the 0x1n finger marker that must be object byte 1.
		 */
		for (i = 29; i <= 35 && i + 1 < len; i++) {
			if ((data[i + 1] & 0xf0) == 0x10) {
				base = i;
				break;
			}
		}

		/*
		 * Accept a partial trailing object (>= 7 bytes covers index,
		 * type, x, y and width).  A frame whose prefix slipped by one
		 * byte leaves only 11 bytes for its single object; requiring
		 * the full 12 used to drop it and emit a bogus finger-up,
		 * which showed up as extra keypresses on the touch keyboard.
		 */
		for (off = base; off + 7 <= (unsigned int)len; off += obj_size) {
			const u8 *o = data + off;
			u8 finger = o[1];
			unsigned int idx = finger & 0x0f;

			/*
			 * Only real fingers carry type nibble 0x1; this also
			 * drops 0x01/0x80 objects that leak in from a slipped
			 * frame.
			 */
			if ((finger & 0xf0) == 0x10 && idx < TCM_MAX_SLOTS) {
				obj_status[idx] = TCM_OBJ_FINGER;
				obj_x[idx] = o[2] | (o[3] << 8);
				obj_y[idx] = o[4] | (o[5] << 8);
				obj_width[idx] = o[6];
				active_objects++;
			}
		}
	} else if (hw->touch_config && hw->touch_config_size > 0) {
		/*
		 * Config-driven parse, ported from the vendor's
		 * syna_tcm_parse_touch_report(): walk the report config and
		 * pull each field out of the report bitstream.
		 */
		const u8 *cfg = hw->touch_config;
		unsigned int cfg_size = hw->touch_config_size;
		unsigned int idx = 0, offset = 0;
		unsigned int objn = 0, next = 0, objects = 0;
		unsigned int active_cnt = 0, end_of_foreach = 0;
		bool active_only = false, has_active_cnt = false;

		while (idx < cfg_size) {
			u8 code = cfg[idx++];
			unsigned int bits, val;

			switch (code) {
			case TOUCH_END:
				goto done;

			case TOUCH_FOREACH_ACTIVE_OBJECT:
				objn = 0;
				next = idx;
				active_only = true;
				break;

			case TOUCH_FOREACH_OBJECT:
				objn = 0;
				next = idx;
				active_only = false;
				break;

			case TOUCH_FOREACH_END:
				end_of_foreach = idx;
				if (active_only) {
					if (has_active_cnt) {
						objects++;
						objn++;
						if (objects < active_cnt)
							idx = next;
					} else if (offset < (unsigned int)len * 8) {
						objn++;
						idx = next;
					}
				}
				/* A FOREACH_END without an open FOREACH is a
				 * no-op (the IC's default config starts with
				 * one) -- just continue. */
				break;

			case TOUCH_PAD_TO_NEXT_BYTE:
				offset = (offset + 7) & ~7u;
				break;

			case TOUCH_OBJECT_N_INDEX:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				if (val < TCM_MAX_SLOTS) {
					objn = val;
					obj_status[val] = 0;
				}
				break;

			case TOUCH_OBJECT_N_CLASSIFICATION:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				if (val && objn < TCM_MAX_SLOTS) {
					obj_status[objn] = TCM_OBJ_FINGER;
					active_objects++;
				}
				break;

			case TOUCH_OBJECT_N_X_POSITION:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				if (objn < TCM_MAX_SLOTS)
					obj_x[objn] = val;
				break;

			case TOUCH_OBJECT_N_Y_POSITION:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				if (objn < TCM_MAX_SLOTS)
					obj_y[objn] = val;
				break;

			case TOUCH_OBJECT_N_X_WIDTH:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				if (objn < TCM_MAX_SLOTS)
					obj_width[objn] =
						val > 255 ? 255 : val;
				break;

			case TOUCH_OBJECT_N_Y_WIDTH:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				if (objn < TCM_MAX_SLOTS &&
				    val > obj_width[objn])
					obj_width[objn] =
						val > 255 ? 255 : val;
				break;

			case TOUCH_NUM_OF_ACTIVE_OBJECTS:
				bits = cfg[idx++];
				val = tcm_get_bits(data, offset, bits);
				offset += bits;
				/*
				 * The S3910's default config reports this
				 * field as 0 even when objects follow, so
				 * drive the foreach loop from the report
				 * size instead (see FOREACH_END).
				 */
				active_cnt = val;
				has_active_cnt = false;
				break;

			default:
				/* field with explicit width we don't use */
				bits = cfg[idx++];
				offset += bits;
				break;
			}
		}
	} else {
		goto done;
	}

done:
	for (obj = 0; obj < TCM_MAX_SLOTS; obj++) {
		input_mt_slot(hw->input, obj);

		if (obj_status[obj] == TCM_OBJ_FINGER) {
			u16 x = obj_x[obj];
			u16 y = obj_y[obj];

			/*
			 * Raw positions are in the sensor range (fw_max, from
			 * the DTS panel-coords).  Scale to the panel pixels
			 * (panel_max, from display-coords).
			 */
			if (hw->fw_max_x && hw->panel_max_x &&
			    hw->fw_max_x != hw->panel_max_x)
				x = (u32)x * hw->panel_max_x / hw->fw_max_x;
			if (hw->fw_max_y && hw->panel_max_y &&
			    hw->fw_max_y != hw->panel_max_y)
				y = (u32)y * hw->panel_max_y / hw->fw_max_y;

			if (hw->cal_x_max > hw->cal_x_min && hw->panel_max_x) {
				u32 v = (x > hw->cal_x_min) ?
					x - hw->cal_x_min : 0;

				v = v * hw->panel_max_x /
					(hw->cal_x_max - hw->cal_x_min);
				x = (v > hw->panel_max_x) ? hw->panel_max_x : v;
			}
			if (hw->cal_y_max > hw->cal_y_min && hw->panel_max_y) {
				u32 v = (y > hw->cal_y_min) ?
					y - hw->cal_y_min : 0;

				v = v * hw->panel_max_y /
					(hw->cal_y_max - hw->cal_y_min);
				y = (v > hw->panel_max_y) ? hw->panel_max_y : v;
			}

			input_mt_report_slot_state(hw->input,
				MT_TOOL_FINGER, true);
			input_report_abs(hw->input, ABS_MT_POSITION_X, x);
			input_report_abs(hw->input, ABS_MT_POSITION_Y, y);
			input_report_abs(hw->input,
					 ABS_MT_TOUCH_MAJOR, obj_width[obj]);
			input_report_abs(hw->input,
					 ABS_MT_WIDTH_MAJOR, obj_width[obj]);
		} else {
			input_mt_report_slot_state(hw->input,
				MT_TOOL_FINGER, false);
		}
	}

	input_report_key(hw->input, BTN_TOUCH, active_objects > 0);
	input_sync(hw->input);
}

/* ---- Firmware loading ---- */

struct fw_area {
	const u8 *data;
	u32 size;
	u32 flash_addr;
};

struct fw_img {
	struct fw_area app_code;
	struct fw_area app_config;
};

static int tcm_get_boot_info(struct tcm_hw *hw, u8 *buf, unsigned int bufsize)
{
	int ret;

	ret = tcm_send_cmd(hw, TCM_CMD_GET_BOOT_INFO, NULL, 0);
	if (ret < 0)
		return ret;

	ret = tcm_read_cmd_response(hw, hw->buf, TCM_MSG_MAX);
	if (ret < 0)
		return ret;

	/* Copy to caller's buffer */
	memcpy(buf, hw->buf, min_t(unsigned int, TCM_MSG_MAX, bufsize));
	return 0;
}

static int tcm_enter_bl(struct tcm_hw *hw)
{
	int ret, retry;
	u8 *buf = hw->buf;

	ret = tcm_send_cmd(hw, TCM_CMD_RUN_BL_FW, NULL, 0);
	if (ret < 0)
		return ret;

	/* Wait for mode change — IC sends REPORT_IDENTIFY or status */
	msleep(200);

	/* Drain any pending responses */
	for (retry = 0; retry < 20; retry++) {
		unsigned long t = jiffies + msecs_to_jiffies(200);

		while (time_before(jiffies, t)) {
			if (!gpiod_get_value(hw->irq))
				break;
			usleep_range(500, 1000);
		}
		if (gpiod_get_value(hw->irq))
			break;
		ret = tcm_read_response(hw, buf, TCM_MSG_MAX);
		if (ret < 0)
			break;
		pr_debug("syna-tcm: bl drain code=0x%02x\n", buf[1]);
	}

	/* Re-identify to confirm bootloader mode */
	ret = tcm_identify(hw);
	if (ret == 0 && hw->firmware_mode == MODE_BOOTLOADER) {
		pr_info("syna-tcm: entered bootloader mode\n");
		return 0;
	}

	pr_warn("syna-tcm: enter_bl: mode=0x%02x (expected 0x%02x)\n",
		hw->firmware_mode, MODE_BOOTLOADER);
	return (hw->firmware_mode == MODE_BOOTLOADER) ? 0 : -EINVAL;
}

static int tcm_erase_flash(struct tcm_hw *hw, u32 page_start, u32 page_count)
{
	u8 payload[4];
	u16 len;

	if (page_start <= 0xff && page_count <= 0xff) {
		payload[0] = page_start & 0xff;
		payload[1] = page_count & 0xff;
		len = 2;
	} else {
		payload[0] = page_start & 0xff;
		payload[1] = (page_start >> 8) & 0xff;
		payload[2] = page_count & 0xff;
		payload[3] = (page_count >> 8) & 0xff;
		len = 4;
	}

	return tcm_send_cmd(hw, TCM_CMD_ERASE_FLASH, payload, len);
}

static int tcm_write_flash(struct tcm_hw *hw, u32 flash_addr,
			   const u8 *data, u32 size,
			   u32 write_block_size, u32 max_payload)
{
	u8 *msg;
	u32 offset;
	u32 chunk;
	u32 chunk_max;
	int ret;

	if (write_block_size == 0)
		return -EINVAL;

	msg = kzalloc(TCM_MSG_MAX, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	/*
	 * Vendor syna_tcm_write_flash():
	 *   w_length = max_wr_size - 8, rounded down to a write block,
	 *   then capped by the device's max write payload size.
	 */
	chunk_max = (hw->max_write_size ?: 1024) - 8;
	chunk_max -= chunk_max % write_block_size;
	if (max_payload && chunk_max > max_payload)
		chunk_max = max_payload;
	if (chunk_max == 0 || chunk_max > TCM_MSG_MAX - 2) {
		kfree(msg);
		return -EINVAL;
	}

	for (offset = 0; offset < size; offset += chunk) {
		u32 block_addr;

		chunk = size - offset;
		if (chunk > chunk_max)
			chunk = chunk_max;

		/* Payload: [block_addr_lo] [block_addr_hi] [data...] */
		block_addr = (flash_addr + offset) / write_block_size;
		msg[0] = block_addr & 0xff;
		msg[1] = (block_addr >> 8) & 0xff;
		memcpy(msg + 2, data + offset, chunk);

		ret = tcm_write_cmd(hw, TCM_CMD_WRITE_FLASH, msg, chunk + 2);
		if (ret < 0) {
			pr_err("syna-tcm: write_flash failed at offset %u\n",
			       offset);
			kfree(msg);
			return ret;
		}

		usleep_range(5000, 10000);
	}

	kfree(msg);
	return 0;
}

static int tcm_parse_fw_image(const u8 *image, u32 image_size,
			      struct fw_img *img)
{
	u32 magic, num_areas, offset, i;
	u32 addr, area_len, area_flash_addr, area_checksum;
	const u8 *area_data;
	const u8 *hdr;

	if (image_size < 8)
		return -EINVAL;

	magic = image[0] | (image[1] << 8) | (image[2] << 16) | (image[3] << 24);
	if (magic != FW_IMG_MAGIC) {
		pr_err("syna-tcm: bad fw magic 0x%08x\n", magic);
		return -EINVAL;
	}

	num_areas = image[4] | (image[5] << 8) | (image[6] << 16) | (image[7] << 24);
	offset = 8; /* past header */

	for (i = 0; i < num_areas; i++) {
		if (offset + 4 > image_size)
			break;
		addr = image[offset] | (image[offset+1] << 8) |
		       (image[offset+2] << 16) | (image[offset+3] << 24);
		offset += 4;

		if (addr + FW_AREA_HDR_SIZE > image_size)
			continue;
		hdr = image + addr;

		magic = hdr[0] | (hdr[1] << 8) | (hdr[2] << 16) | (hdr[3] << 24);
		if (magic != FW_AREA_MAGIC)
			continue;

		/*
		 * area_descriptor (vendor synaptics_touchcom_func_base_flash.h):
		 *   [0-3]   magic
		 *   [4-19]  id_string (16 bytes)
		 *   [20-23] flags
		 *   [24-27] flash_addr_words (LE32, in 16-bit words)
		 *   [28-31] length (LE32, bytes)
		 *   [32-35] checksum
		 */
		area_len = hdr[28] | (hdr[29] << 8) | (hdr[30] << 16) |
			   (hdr[31] << 24);
		area_flash_addr = hdr[24] | (hdr[25] << 8) |
				  (hdr[26] << 16) | (hdr[27] << 24);
		area_flash_addr *= 2; /* words to bytes */
		area_checksum = hdr[32] | (hdr[33] << 8) |
				(hdr[34] << 16) | (hdr[35] << 24);
		area_data = hdr + FW_AREA_HDR_SIZE;

		if (addr + FW_AREA_HDR_SIZE + area_len > image_size)
			continue;

		if (memcmp(hdr + 4, "APP_CODE", 8) == 0) {
			img->app_code.data = area_data;
			img->app_code.size = area_len;
			img->app_code.flash_addr = area_flash_addr;
			pr_info("syna-tcm: fw APP_CODE: addr=0x%08x size=%u\n",
				area_flash_addr, area_len);
		} else if (memcmp(hdr + 4, "APP_CONFIG", 10) == 0) {
			img->app_config.data = area_data;
			img->app_config.size = area_len;
			img->app_config.flash_addr = area_flash_addr;
			pr_info("syna-tcm: fw APP_CONFIG: addr=0x%08x size=%u\n",
				area_flash_addr, area_len);
		}
	}

	if (!img->app_code.data || img->app_code.size == 0) {
		pr_err("syna-tcm: no APP_CODE in firmware image\n");
		return -EINVAL;
	}

	return 0;
}

static int tcm_load_firmware(struct tcm_hw *hw)
{
	const struct firmware *fw_entry = NULL;
	const u8 *fw_data;
	u32 fw_size;
	struct fw_img img;
	u8 *boot_buf;
	u8 *p;
	int ret;
	u32 write_block_words, erase_page_words, max_payload;
	u32 write_block_size, page_size;
	u32 page_start, page_count;
	u32 block_addr;

	boot_buf = kmalloc(TCM_MSG_MAX, GFP_KERNEL);
	if (!boot_buf)
		return -ENOMEM;

	ret = request_firmware(&fw_entry,
			       "tp/24618/FW_S3910_TIANMA_HBP.img",
			       &hw->spi->dev);
	if (ret) {
		pr_err("syna-tcm: request_firmware failed: %d\n", ret);
		kfree(boot_buf);
		return ret;
	}

	pr_info("syna-tcm: firmware loaded %zu bytes\n", fw_entry->size);

	/* Skip FwUp wrapper (0x50 bytes) if present */
	fw_data = fw_entry->data;
	fw_size = fw_entry->size;
	if (fw_size > FW_UP_OFFSET &&
	    fw_data[0] == 'F' && fw_data[1] == 'w' &&
	    fw_data[2] == 'U' && fw_data[3] == 'p') {
		fw_data += FW_UP_OFFSET;
		fw_size -= FW_UP_OFFSET;
		pr_info("syna-tcm: skipped FwUp wrapper, %u bytes remain\n",
			fw_size);
	}

	memset(&img, 0, sizeof(img));
	ret = tcm_parse_fw_image(fw_data, fw_size, &img);
	if (ret) {
		pr_err("syna-tcm: firmware parse failed: %d\n", ret);
		goto release_fw;
	}

	/* Step 1: Enter bootloader mode */
	if (hw->firmware_mode != MODE_BOOTLOADER) {
		pr_info("syna-tcm: entering bootloader mode (current=0x%02x)\n",
			hw->firmware_mode);
		ret = tcm_enter_bl(hw);
		if (ret) {
			pr_err("syna-tcm: failed to enter bootloader: %d\n", ret);
			goto release_fw;
		}
	}

	/* Step 2: Get boot info */
	ret = tcm_get_boot_info(hw, boot_buf, TCM_MSG_MAX);
	if (ret) {
		pr_err("syna-tcm: get_boot_info failed: %d\n", ret);
		goto release_fw;
	}

	p = boot_buf + TCM_HEADER_SIZE;
	/*
	 * Vendor struct tcm_boot_info:
	 *   [0]   version
	 *   [1]   status
	 *   [2-3] asic_id
	 *   [4]   write_block_size_words
	 *   [5-6] erase_page_size_words (LE16)
	 *   [7-8] max_write_payload_size (LE16)
	 */
	write_block_words = p[4];
	erase_page_words = p[5] | (p[6] << 8);
	max_payload = p[7] | (p[8] << 8);

	write_block_size = write_block_words * 2;
	page_size = erase_page_words * 2;

	pr_info("syna-tcm: boot_info: wr_blk=%u page=%u max_payload=%u\n",
		write_block_size, page_size, max_payload);

	if (page_size == 0 || write_block_size == 0) {
		pr_err("syna-tcm: invalid boot_info\n");
		ret = -EINVAL;
		goto release_fw;
	}

	/* Step 3: Erase and write APP_CODE */
	if (img.app_code.size > 0) {
		page_start = img.app_code.flash_addr / page_size;
		page_count = (img.app_code.size + page_size - 1) / page_size;
		pr_info("syna-tcm: erasing APP_CODE: page %u +%u\n",
			page_start, page_count);

		ret = tcm_erase_flash(hw, page_start, page_count);
		if (ret) {
			pr_err("syna-tcm: erase APP_CODE failed: %d\n", ret);
			goto release_fw;
		}
		msleep(500);

		block_addr = img.app_code.flash_addr / write_block_size;
		pr_info("syna-tcm: writing APP_CODE: block %u, %u bytes\n",
			block_addr, img.app_code.size);

		ret = tcm_write_flash(hw, img.app_code.flash_addr,
				       img.app_code.data, img.app_code.size,
				       write_block_size, max_payload);
		if (ret) {
			pr_err("syna-tcm: write APP_CODE failed: %d\n", ret);
			goto release_fw;
		}
	}

	/* Step 4: Erase and write APP_CONFIG */
	if (img.app_config.size > 0) {
		page_start = img.app_config.flash_addr / page_size;
		page_count = (img.app_config.size + page_size - 1) / page_size;
		pr_info("syna-tcm: erasing APP_CONFIG: page %u +%u\n",
			page_start, page_count);

		ret = tcm_erase_flash(hw, page_start, page_count);
		if (ret) {
			pr_err("syna-tcm: erase APP_CONFIG failed: %d\n", ret);
			goto release_fw;
		}
		msleep(500);

		block_addr = img.app_config.flash_addr / write_block_size;
		pr_info("syna-tcm: writing APP_CONFIG: block %u, %u bytes\n",
			block_addr, img.app_config.size);

		ret = tcm_write_flash(hw, img.app_config.flash_addr,
				       img.app_config.data, img.app_config.size,
				       write_block_size, max_payload);
		if (ret) {
			pr_err("syna-tcm: write APP_CONFIG failed: %d\n", ret);
			goto release_fw;
		}
	}

	/* Step 5: Reset to load new firmware */
	pr_info("syna-tcm: firmware flash complete, resetting\n");
	ret = tcm_send_cmd(hw, TCM_CMD_RESET, NULL, 0);
	msleep(500);

release_fw:
	release_firmware(fw_entry);
	kfree(boot_buf);
	return ret;
}

/* ---- IRQ handler ---- */

static irqreturn_t tcm_irq_handler(int irq, void *data)
{
	struct tcm_hw *hw = data;
	int ret;
	u16 payload_len;
	u8 *buf = hw->irq_buf;

	ret = tcm_read_response(hw, buf, TCM_MSG_MAX);
	if (ret < 0)
		return IRQ_HANDLED;

	payload_len = buf[2] | (buf[3] << 8);

	if (buf[1] == TCM_REPORT_TOUCH) {
		tcm_parse_touch(hw, buf + TCM_HEADER_SIZE, payload_len);
	} else if (buf[1] == TCM_REPORT_IDENTIFY) {
		pr_info("syna-tcm: device reset in IRQ\n");
	} else {
		pr_info("syna-tcm: irq report 0x%02x len=%u\n",
			buf[1], payload_len);
	}

	return IRQ_HANDLED;
}

/* ---- SPI driver probe ---- */

static int tcm_probe(struct spi_device *spi)
{
	struct tcm_hw *hw;
	struct input_dev *input;
	int ret;

	pr_info("syna-tcm: probe enter\n");

	hw = devm_kzalloc(&spi->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	hw->buf = kmalloc(TCM_MSG_MAX, GFP_KERNEL);
	if (!hw->buf)
		return -ENOMEM;

	hw->irq_buf = kmalloc(TCM_MSG_MAX, GFP_KERNEL);
	if (!hw->irq_buf) {
		kfree(hw->buf);
		return -ENOMEM;
	}

	hw->use_default_format = true;

	hw->spi = spi;
	spi_set_drvdata(spi, hw);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret) {
		pr_err("syna-tcm: spi_setup failed: %d\n", ret);
		return ret;
	}

	hw->avdd = devm_regulator_get(&spi->dev, "avdd");
	if (IS_ERR(hw->avdd))
		return dev_err_probe(&spi->dev, PTR_ERR(hw->avdd),
				     "no avdd regulator\n");

	ret = regulator_enable(hw->avdd);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "avdd enable failed\n");

	hw->reset = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(hw->reset)) {
		ret = dev_err_probe(&spi->dev, PTR_ERR(hw->reset),
				    "no reset GPIO\n");
		goto err_reg_disable;
	}

	hw->irq = devm_gpiod_get(&spi->dev, "irq", GPIOD_IN);
	if (IS_ERR(hw->irq)) {
		ret = dev_err_probe(&spi->dev, PTR_ERR(hw->irq),
				    "no irq GPIO\n");
		goto err_reg_disable;
	}

	hw->irq_num = of_irq_get(spi->dev.of_node, 0);
	if (hw->irq_num < 0) {
		ret = hw->irq_num;
		pr_err("syna-tcm: platform_get_irq failed: %d\n", ret);
		goto err_reg_disable;
	}

	/* Reset the controller - active-low: LOW=assert, HIGH=release */
	pr_info("syna-tcm: before reset: ATTN=%d RESET=%d\n",
		gpiod_get_value(hw->irq), gpiod_get_value(hw->reset));
	gpiod_set_value(hw->reset, 0);
	usleep_range(10000, 15000);
	gpiod_set_value(hw->reset, 1);
	msleep(80);
	pr_info("syna-tcm: after reset: ATTN=%d\n",
		gpiod_get_value(hw->irq));

	/*
	 * Vendor probe sequence for TCM v1:
	 *  1. Send 0x07 (CMD_TCM2_ACK magic) raw to trigger IC output
	 *  2. Read 4 bytes -- should start with 0xa5 (TCM v1 marker)
	 *  3. If REPORT_IDENTIFY (0x10): read 28-byte identify payload
	 *  4. Only send CMD_IDENTIFY if boot report was NOT REPORT_IDENTIFY
	 */
	{
		u8 *boot_buf = hw->buf;
		u8 hdr[TCM_HEADER_SIZE + 1];
		u8 magic = TCM_DETECT_MAGIC;
		int got_identify = 0;

		/* Step 1: Send 0x07 trigger */
		ret = tcm_spi_write(hw, &magic, 1);
		pr_info("syna-tcm: sent 0x07 trigger, ret=%d\n", ret);
		msleep(20);

		/* Step 2: Wait for ATTN, then read 4-byte header */
		{
			unsigned long t = jiffies + msecs_to_jiffies(500);
			while (time_before(jiffies, t)) {
				if (!gpiod_get_value(hw->irq))
					break;
				usleep_range(500, 1000);
			}
		}

		if (!gpiod_get_value(hw->irq)) {
			int retry;
			u8 saved[2];
			int saved_cnt = 0;

			for (retry = 0; retry < 10; retry++) {
				ret = tcm_spi_read(hw, hdr,
						   TCM_HEADER_SIZE + 1);
				if (ret < 0)
					break;

				if (hdr[0] == TCM_V1_MARKER) {
					saved[0] = hdr[TCM_HEADER_SIZE];
					saved_cnt = 1;
					break;
				}

				if (hdr[0] == TCM_REPORT_IDENTIFY) {
					saved[0] = hdr[3];
					saved[1] = hdr[4];
					saved_cnt = 2;
					memmove(hdr + 1, hdr, 3);
					hdr[0] = TCM_V1_MARKER;
					break;
				}

				pr_warn("syna-tcm: boot bad marker: %02x %02x %02x %02x %02x (retry %d)\n",
					hdr[0], hdr[1], hdr[2], hdr[3],
					hdr[4], retry);
				usleep_range(1000, 2000);
			}

			if (ret == 0 && hdr[0] == TCM_V1_MARKER) {
				u16 plen;

				plen = hdr[2] | (hdr[3] << 8);
				pr_info("syna-tcm: TCM v1 boot, code=0x%02x len=%u (saved=%d)\n",
					hdr[1], plen, saved_cnt);

				if (hdr[1] == TCM_REPORT_IDENTIFY &&
				    plen + TCM_HEADER_SIZE + 10 <= TCM_MSG_MAX) {
					memcpy(boot_buf + TCM_HEADER_SIZE,
					       saved, saved_cnt);
					ret = tcm_spi_read(hw,
						boot_buf + TCM_HEADER_SIZE +
						saved_cnt,
						plen + 10 - saved_cnt);
					if (ret == 0) {
						memcpy(boot_buf, hdr,
						       TCM_HEADER_SIZE);
						got_identify = 1;
						tcm_parse_identify(hw,
								  boot_buf);
					}
				}
			} else {
				pr_warn("syna-tcm: no TCM v1 marker\n");
			}
		} else {
			pr_info("syna-tcm: no boot report (ATTN still high)\n");
		}
	}

	/* Get firmware info */
	ret = tcm_identify(hw);
	if (ret)
		goto err_reg_disable;

	/*
	 * An unexpected mode almost always means the controller was left in a
	 * half-state by a previous probe (init rebinds SPI6).  Do NOT touch
	 * its flash -- a transient IDENTIFY glitch must never erase the
	 * running firmware.  A hardware reset + detect trigger brings it back
	 * into application mode deterministically.
	 */
	if (hw->firmware_mode != MODE_APPLICATION &&
	    hw->firmware_mode != MODE_APPLICATION_03) {
		pr_warn("syna-tcm: unexpected mode 0x%02x, resetting controller\n",
			hw->firmware_mode);
		gpiod_set_value(hw->reset, 0);
		usleep_range(10000, 15000);
		gpiod_set_value(hw->reset, 1);
		msleep(80);
		{
			u8 magic = TCM_DETECT_MAGIC;

			tcm_spi_write(hw, &magic, 1);
			msleep(20);
		}
		tcm_identify(hw);

		if (hw->firmware_mode != MODE_APPLICATION &&
		    hw->firmware_mode != MODE_APPLICATION_03) {
			if (!flash_firmware) {
				pr_warn("syna-tcm: refusing to reflash firmware (mode=0x%02x); set syna_tcm_spi.flash_firmware=1 to allow\n",
					hw->firmware_mode);
			} else {
				pr_info("syna-tcm: mode=0x%02x, loading firmware\n",
					hw->firmware_mode);
				ret = tcm_load_firmware(hw);
				if (ret == 0) {
					gpiod_set_value(hw->reset, 0);
					usleep_range(10000, 15000);
					gpiod_set_value(hw->reset, 1);
					msleep(200);

					{
						u8 magic = TCM_DETECT_MAGIC;
						tcm_spi_write(hw, &magic, 1);
						msleep(20);
					}

					ret = tcm_identify(hw);
					if (ret == 0)
						pr_info("syna-tcm: after fw load, mode=0x%02x\n",
							hw->firmware_mode);
				}
				if (hw->firmware_mode != MODE_APPLICATION &&
				    hw->firmware_mode != MODE_APPLICATION_03)
					pr_warn("syna-tcm: still not in app mode (0x%02x)\n",
						hw->firmware_mode);
			}
		}
	}

	ret = tcm_get_app_info(hw);
	if (ret)
		goto err_reg_disable;

	/*
	 * Vendor-style coordinate calibration.  As in the vendor DTS,
	 * "panel-coords" is the raw sensor range (e.g. 20480x44800, 16x the
	 * panel pixels) and "display-coords" is the panel resolution.  The IC
	 * reports positions in the raw range, so scaling raw*display/panel
	 * gives pixels.  The app_info max_x/max_y are not reliable here and
	 * are overridden when the DTS provides panel-coords.
	 */
	{
		u32 raw[2], disp[2], cal[4];

		if (of_property_read_u32_array(spi->dev.of_node,
					       "touchpanel,panel-coords",
					       raw, 2) == 0) {
			hw->fw_max_x = raw[0];
			hw->fw_max_y = raw[1];
		}
		if (of_property_read_u32_array(spi->dev.of_node,
					       "touchpanel,display-coords",
					       disp, 2) == 0) {
			hw->panel_max_x = disp[0];
			hw->panel_max_y = disp[1];
		}
		if (of_property_read_u32_array(spi->dev.of_node,
					       "touchpanel,cal-coords",
					       cal, 4) == 0) {
			hw->cal_x_min = cal[0];
			hw->cal_x_max = cal[1];
			hw->cal_y_min = cal[2];
			hw->cal_y_max = cal[3];
		}
		pr_info("syna-tcm: raw=%ux%u display=%ux%u cal x %u..%u y %u..%u\n",
			hw->fw_max_x, hw->fw_max_y,
			hw->panel_max_x, hw->panel_max_y,
			hw->cal_x_min, hw->cal_x_max,
			hw->cal_y_min, hw->cal_y_max);
	}

	/*
	 * The report descriptor the IC returns describes the *configuration*
	 * layout, not the HBP active-frame (0x23/0x11) records this driver
	 * consumes.  Decoding the actual frames shows they are the
	 * byte-oriented 32-byte-prefix + 12-byte objects handled by the
	 * default parser, so the config is only fetched to leave the IC in a
	 * known state.  Do this before registering the IRQ so the response is
	 * read by polling here.
	 */
	tcm_get_touch_report_config(hw);

	/* Register input device */
	input = devm_input_allocate_device(&spi->dev);
	if (!input) {
		ret = -ENOMEM;
		goto err_reg_disable;
	}

	input->name = "synaptics-tcm";
	input->phys = "synaptics-tcm/spi";
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x06cb;
	input->id.product = 0x3910;
	input->id.version = 0x0001;

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			     hw->panel_max_x ?: 1280, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			     hw->panel_max_y ?: 2800, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_mt_init_slots(input, hw->max_objects ?: TCM_MAX_SLOTS,
			    INPUT_MT_DIRECT);

	ret = input_register_device(input);
	if (ret) {
		pr_err("syna-tcm: input_register failed: %d\n", ret);
		goto err_reg_disable;
	}

	hw->input = input;

	ret = devm_request_threaded_irq(&spi->dev, hw->irq_num, NULL,
					tcm_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"synaptics-tcm", hw);
	if (ret) {
		pr_err("syna-tcm: irq request failed: %d\n", ret);
		goto err_input_unreg;
	}

	/*
	 * The vendor daemon enables the HBP active frame report (0x23)
	 * with ENABLE_REPORT.  Touch reports (0x11) then flow.
	 * Try it — without a userspace daemon the kernel must do this.
	 */
	{
		u8 rpt = TCM_REPORT_HBP_ACTIVE_FRAME;

		tcm_send_cmd(hw, TCM_CMD_ENABLE_REPORT, &rpt, 1);
		msleep(50);
	}

	pr_info("syna-tcm: probe ok: fw=%dx%d panel=%dx%d, %d obj, IRQ %d ATTN=%d\n",
		hw->fw_max_x, hw->fw_max_y,
		hw->panel_max_x, hw->panel_max_y,
		hw->max_objects, hw->irq_num,
		gpiod_get_value(hw->irq));

	return 0;

err_input_unreg:
	input_unregister_device(input);
err_reg_disable:
	regulator_disable(hw->avdd);
	return ret;
}

static void tcm_remove(struct spi_device *spi)
{
	struct tcm_hw *hw = spi_get_drvdata(spi);

	tcm_send_cmd(hw, TCM_CMD_DISABLE_REPORT, NULL, 0);
	regulator_disable(hw->avdd);
}

static const struct of_device_id tcm_of_match[] = {
	{ .compatible = "synaptics,tcm-spi-hbp" },
	{ .compatible = "synaptics,s3910" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcm_of_match);

static const struct spi_device_id tcm_id_table[] = {
	{ "synaptics-tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, tcm_id_table);

static struct spi_driver tcm_driver = {
	.driver = {
		.name = "synaptics-tcm-spi",
		.of_match_table = tcm_of_match,
	},
	.probe = tcm_probe,
	.remove = tcm_remove,
	.id_table = tcm_id_table,
};
module_spi_driver(tcm_driver);

MODULE_DESCRIPTION("Synaptics TCM1 SPI touchscreen driver");
MODULE_LICENSE("GPL");
