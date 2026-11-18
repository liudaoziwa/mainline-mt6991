// SPDX-License-Identifier: GPL-2.0
/*
 * SCP sensorhub probe for the MT6991 (RMX6688) mainline port.
 *
 * The stock scp.img firmware contains the MTK/OPLUS sensorhub middleware
 * and the bmi2xy / tcs3720 / akm sensor drivers.  This module exercises
 * that middleware over the scp.ko IPI channels:
 *
 *   1. registers IPI_IN_SENSOR_CTRL (30) and IPI_IN_SENSOR_NOTIFY (32)
 *   2. on the scp.ko ready event sends SENS_COMM_NOTIFY_READY_CMD on
 *      IPI_OUT_SENSOR_NOTIFY (31), exactly like the vendor ready.c
 *   3. once the firmware reports its own READY, configures the DRAM
 *      share-memory rings and sends SENS_COMM_CTRL_SHARE_MEMORY_CMD.
 *      Which rings are announced is controlled by the shm_mask module
 *      parameter (bit per payload_type table entry).  The firmware
 *      asserts as soon as it tries to push data with no ring configured,
 *      so the default announces only the data ring (bit 0) - the minimal
 *      set needed for accelerometer samples.
 *   4. sends a TIMESYNC, then enables the accelerometer via
 *      SENS_COMM_CTRL_ENABLE_CMD with the stock batch config
 *   5. on every DATA notify reads the sensor records out of the data
 *      ring (write_position) and prints the first few plus every 200th
 *   6. 30 s later disables the accelerometer again
 *
 * Firmware logging uses the stock SCP logger protocol (vendor rv/scp_logger.c):
 * the AP initialises the DRAM log control block (SCP_A_LOGGER_MEM_ID), sends
 * SCP_LOGGER_IPI_INIT on IPI_OUT_LOGGER_CTRL (17), the firmware answers with
 * struct SCP_LOG_INFO through IPI_IN_LOGGER_CTRL (21) giving its TCM log ring
 * location, and the AP writes the DRAM log address into the firmware-announced
 * TCM slot.  ASCII strings are then dumped from the DRAM log region and from
 * the announced TCM ring window (never from unannounced TCM areas - those may
 * be protected and reading them aborted the AP kernel once) at: before the
 * share-mem config, when the SCP_STOP event is received (post-crash firmware
 * log) and after the test window.
 *
 * Frame layouts, command values, ring setup and the crc8 implementation
 * are copied from the vendor stack
 * (drivers/misc/mediatek/sensor/2.0/sensorhub/).
 */

#define pr_fmt(fmt) "sensorhub_probe: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/sched/clock.h>
#include <linux/rtc.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <asm/arch_timer.h>
#include <asm/barrier.h>
#include <linux/unaligned.h>
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>
#include "scp.h"

enum {
	SENS_COMM_CTRL_DISABLE_CMD = 0,
	SENS_COMM_CTRL_ENABLE_CMD = 1,
	SENS_COMM_CTRL_TIMESYNC_CMD = 10,
	SENS_COMM_CTRL_SHARE_MEMORY_CMD = 11,
	SENS_COMM_NOTIFY_DATA_CMD = 0,
	SENS_COMM_NOTIFY_SUPER_DATA_CMD = 2,
	SENS_COMM_NOTIFY_SUPER_FULL_CMD = 3,
	SENS_COMM_NOTIFY_READY_CMD = 4,
	SENS_COMM_NOTIFY_LIST_CMD = 5,
};

enum {
	SENSOR_TYPE_INVALID = 0,
	SENSOR_TYPE_ACCELEROMETER = 1,
	SENSOR_TYPE_MAGNETIC_FIELD = 2,
	SENSOR_TYPE_GYROSCOPE = 4,
	SENSOR_TYPE_LIGHT = 5,
	SENSOR_TYPE_PROXIMITY = 8,
	SENSOR_TYPE_LUX_AOD = 78,
	SENSOR_TYPE_CCT = 95,
};

struct sensor_comm_ctrl {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	u8 length;
	u8 crc8;
	u8 data[] __aligned(4);
} __packed __aligned(4);

struct sensor_comm_ack {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	s8 ret_val;
	u8 crc8;
} __packed __aligned(4);

struct sensor_comm_notify {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	u8 length;
	u8 crc8;
	s32 value[5] __aligned(4);
} __packed __aligned(4);

struct sensor_comm_batch {
	s64 delay;
	s64 latency;
} __packed __aligned(4);

struct sensor_comm_timesync {
	s64 host_timestamp;
	s64 host_archcounter;
	s64 sched_clock;
	s32 usecond;
	s32 second;
	s32 minute;
	s32 hour;
	s32 day;
	s32 month;
} __packed __aligned(4);

struct data_notify {
	s32 write_position;
	s64 scp_timestamp;
	s64 scp_archcounter;
} __packed __aligned(4);

struct sensor_comm_share_mem {
	u8 available_num;
	struct {
		u8 payload_type;
		u32 payload_base;
	} __aligned(4) base_info[4];
} __packed __aligned(4);

struct share_mem_base {
	u32 rp;
	u32 wp;
	u32 buffer_size;
	u32 item_size;
	u8 data[] __aligned(4);
} __packed __aligned(4);

struct share_mem_data {
	u8 sensor_type;
	u8 action;
	u8 accurancy;
	u8 padding[1];
	s64 timestamp;
	s32 value[6] __aligned(4);
} __packed __aligned(4);

struct share_mem_super_data {
	u8 sensor_type;
	u8 action;
	u8 accurancy;
	u8 padding[1];
	s64 timestamp;
	s32 value[16] __aligned(4);
} __packed __aligned(4);

struct share_mem_info {
	u8 sensor_type;
	u8 padding[3];
	u32 gain;
	u8 name[16];
	u8 vendor[16];
} __packed __aligned(4);

struct share_mem_debug {
	u8 sensor_type;
	u8 padding[3];
	u32 written;
	u8 buffer[4032] __aligned(4);
} __packed __aligned(4);

struct share_mem_cmd {
	u8 command;
	u8 tx_len;
	u8 rx_len;
	u8 padding[1];
	s32 data[15] __aligned(4);
} __packed __aligned(4);

/* stock SCP logger protocol (vendor rv/scp_logger.c) */
#define SCP_LOGGER_IPI_INIT	0x4C4F4701
#define SCP_LOGGER_MAGIC	0x504C5402
#define LOGGER_DRAM_BUF_LEN	(1 * 1024 * 1024)
#define LOGGER_LAST_LOG_SIZE	0x10000

/*
 * Firmware logger state (inside the SCP TCM window) and the DRAM region
 * table the firmware's timer task uses to resolve region 3 (the logger
 * carveout).  The table lives in the firmware's DRAM window at VA
 * 0xB0000000: header at window+0x40 (base/total/table offset) and
 * 29 (offset, size) records at window+table offset (NUMS_MEM_ID
 * entries are read by the firmware).  Offsets are relative to the SCP
 * share carveout base, exactly like scp_get_reserve_mem_phys() lay-out.
 */
#define FW_REGION_HDR_OFS	0x40
#define FW_REGION_TBL_OFS	0x80
#define FW_REGION_ENTRIES	29
#define SCP_SHARE_TOTAL		0x14F0000

struct logger_log_ctrl {
	u32 base;
	u32 size;
	u32 enable;
	u32 info_ofs;
	u32 buff_ofs;
	u32 buff_size;
} __packed;

struct logger_buffer_info {
	u32 r_pos;
	u32 w_pos;
} __packed;

struct scp_log_info {
	u32 dram_addr;
	u32 buf_addr;
	u32 start_addr;
	u32 end_addr;
	u32 buf_maxlen;
} __packed;

struct scp_logger_ctrl_msg {
	u32 cmd;
	union {
		struct {
			u32 addr;
			u32 size;
		} init;
		struct {
			u32 enable;
		} flag;
		struct scp_log_info info;
	} u;
} __packed;

static const u8 crc8_table[] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
	0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x00, 0x9d, 0x23, 0xbe, 0x46, 0xdb, 0x65, 0xf8,
	0x8c, 0x11, 0xaf, 0x32, 0xca, 0x57, 0xe9, 0x74
};

static u8 tiny_crc8(u8 *ptr, u8 len)
{
	u8 crc = 0;

	while (len-- > 0) {
		crc = *ptr++ ^ crc;
		crc = crc8_table[crc & 0x0f] ^
			crc8_table[16 + ((crc >> 4) & 0x0f)];
	}

	return crc;
}

#define SHM_NR 6

/* vendor shm_usage_table[]: payload_type enum 0..5 -> reserve mem id */
static const u8 shm_ids[SHM_NR] = {
	SENS_MEM_ID, SENS_SUPER_MEM_ID, SENS_LIST_MEM_ID,
	SENS_DEBUG_MEM_ID, SENS_CUSTOM_W_MEM_ID, SENS_CUSTOM_R_MEM_ID,
};

static const u8 shm_payload_types[SHM_NR] = { 0, 1, 2, 3, 4, 5 };

static const u32 shm_item_sizes[SHM_NR] = {
	sizeof(struct share_mem_data),
	sizeof(struct share_mem_super_data),
	sizeof(struct share_mem_info),
	sizeof(struct share_mem_debug),
	sizeof(struct share_mem_cmd),
	sizeof(struct share_mem_cmd),
};

/* bit i = announce shm_usage_table[i]; default: data + super + list */
static unsigned int shm_mask = 0x07;
module_param(shm_mask, uint, 0444);
MODULE_PARM_DESC(shm_mask, "share-mem rings to announce (bit per payload type)");

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "dump firmware logs and per-sample debug lines");

/* 0 keeps the accelerometer streaming until the SCP is reset */
static unsigned int test_window_s;
module_param(test_window_s, uint, 0644);
MODULE_PARM_DESC(test_window_s, "disable the accelerometer after N seconds (0 = keep enabled)");

#define SCP_TCM_PHYS	0x1c800000
#define SCP_TCM_SIZE	0x360000
#define FWLOG_MAX_LINES	300
#define FWLOG_MIN_RUN	10

static u8 __iomem *shm_base[SHM_NR];
static u8 __iomem *shm_area[SHM_NR];
static phys_addr_t shm_phys[SHM_NR];
static u32 shm_bufsize[SHM_NR];
static u32 shm_fw_off[SHM_NR];
static u32 shm_fw_size[SHM_NR];
static bool shm_fw_tbl_ok;
static u32 data_rp;
static u32 super_rp;
static bool data_wp_bogus;
static unsigned int gyro_samples;

static u8 ctrl_rx[PIN_IN_SIZE_SENSOR_CTRL * MBOX_SLOT_SIZE];
static u8 notify_rx[PIN_IN_SIZE_SENSOR_NOTIFY * MBOX_SLOT_SIZE];
static u8 logger_rx[PIN_IN_SIZE_LOGGER_CTRL * MBOX_SLOT_SIZE];
static u8 __iomem *tcm_map;
static struct scp_log_info logger_info;
static bool logger_info_valid;
static u8 tx_sequence;
static bool platform_ready;
static bool sensor_ready;
static DEFINE_MUTEX(tx_lock);
static atomic_t ctrl_acks = ATOMIC_INIT(0);
static atomic_t notify_count = ATOMIC_INIT(0);
static atomic_t data_notify_count = ATOMIC_INIT(0);
static atomic_t super_notify_count = ATOMIC_INIT(0);
static atomic_t data_samples = ATOMIC_INIT(0);
static int last_ack_ret = -1000;
static struct delayed_work probe_work;
static struct delayed_work disable_accel_work;
static struct work_struct logger_notify_work;
static struct delayed_work fwlog_dump_work;
static struct delayed_work phase2_work;
static struct workqueue_struct *probe_wq;

static void sensorhub_probe_iio_push(const struct share_mem_data *sample);
static void sensorhub_probe_gyro_push(const struct share_mem_data *sample);
static void sensorhub_probe_push_s32(struct iio_dev *indio, const s32 *vals,
				     int count, s64 scp_ts);

static struct platform_device *sensorhub_probe_iio_pdev;
static struct iio_dev *sensorhub_iio;
static struct iio_dev *sensorhub_gyro_iio;
static struct iio_dev *sensorhub_mag_iio;
static struct iio_dev *sensorhub_light_iio;
static struct iio_dev *sensorhub_prox_iio;
static int sensorhub_last[3];
static int sensorhub_gyro_last[3];
static int sensorhub_mag_last[3];
static int sensorhub_light_last;
static int sensorhub_prox_last;
static unsigned int mag_samples;
static unsigned int light_samples;
static unsigned int prox_samples;
static s64 cct_diag_ts;
static s64 lux_diag_ts;
static unsigned int gyro_scale_micro = 1000;
static s64 iio_ts_offset;
static bool iio_ts_offset_valid;

static void sensorhub_probe_map_tcm(void)
{
	if (tcm_map)
		return;
	/*
	 * The firmware may announce its log ring at any time (including
	 * right after an SCP recovery, before probe_work runs), so the TCM
	 * window has to be mapped from process context at module load
	 * instead of lazily from a work item.
	 */
	tcm_map = ioremap(SCP_TCM_PHYS, SCP_TCM_SIZE);
	if (!tcm_map)
		pr_warn("cannot map SCP TCM\n");
	else
		pr_info("tcm mapped 0x%x+0x%x\n", SCP_TCM_PHYS, SCP_TCM_SIZE);
}

static void sensorhub_probe_dump_strings(const char *tag, u8 __iomem *base,
					 u32 size, int maxlines)
{
	char sbuf[160];
	u32 i, run_off = 0;
	int lines = 0, slen = 0;

	for (i = 0; i + 4 <= size && lines < maxlines; i += 4) {
		u32 w = readl_relaxed(base + i);
		int b;

		if (!w || w == 0xffffffff) {
			if (slen >= FWLOG_MIN_RUN) {
				sbuf[min_t(int, slen, (int)sizeof(sbuf) - 1)] = 0;
				pr_info("%s %06x: %s\n", tag, run_off, sbuf);
				lines++;
			}
			slen = 0;
			continue;
		}
		for (b = 0; b < 4; b++) {
			u8 c = (w >> (8 * b)) & 0xff;

			if (c >= 0x20 && c <= 0x7e) {
				if (!slen)
					run_off = i + b;
				if (slen < sizeof(sbuf) - 1)
					sbuf[slen] = c;
				slen++;
			} else if (slen) {
				if (slen >= FWLOG_MIN_RUN) {
					sbuf[min_t(int, slen,
						   (int)sizeof(sbuf) - 1)] = 0;
					pr_info("%s %06x: %s\n", tag, run_off,
						sbuf);
					lines++;
				}
				slen = 0;
			}
			if (lines >= maxlines)
				break;
		}
	}
	pr_info("%s done (%d lines)\n", tag, lines);
}

/*
 * Read the firmware TCM last-log ring.  Only the window announced by the
 * firmware through IPI_IN_LOGGER_CTRL is touched; the ring is copied out
 * with one bounded memcpy_fromio and scanned in RAM afterwards.  The
 * newest entries (just before the write index) are printed first: if the
 * system dies mid-dump, the firmware's last words are already out.
 */
static void sensorhub_probe_dump_tcm_ring(void)
{
	static u8 ring[LOGGER_LAST_LOG_SIZE];
	u32 maxlen, off, end_idx = 0, tail, start;

	if (!tcm_map) {
		pr_info("tcm-log: TCM not mapped\n");
		return;
	}
	if (!logger_info_valid) {
		pr_info("tcm-log: no logger info from firmware yet\n");
		return;
	}
	off = logger_info.buf_addr;
	maxlen = logger_info.buf_maxlen;
	if (!maxlen || maxlen > sizeof(ring) || off > SCP_TCM_SIZE ||
	    maxlen > SCP_TCM_SIZE - off) {
		pr_warn("tcm-log: bad ring info off 0x%x maxlen %u\n",
			off, maxlen);
		return;
	}
	if (logger_info.end_addr + 4 <= SCP_TCM_SIZE)
		end_idx = readl(tcm_map + logger_info.end_addr);
	if (end_idx >= maxlen)
		end_idx = 0;
	pr_info("tcm-log ring: off 0x%x maxlen %u end %u\n", off, maxlen,
		end_idx);
	memcpy_fromio(ring, tcm_map + off, maxlen);

	if (end_idx) {
		tail = min_t(u32, end_idx, 0x800);
		start = end_idx - tail;
		pr_info("tcm-log newest 0x%x..0x%x:\n", start, end_idx);
		sensorhub_probe_dump_strings("tcm-log-new",
					     (u8 __iomem *)(ring + start),
					     tail, 160);
	}
	sensorhub_probe_dump_strings("tcm-log", (u8 __iomem *)ring, maxlen,
				     FWLOG_MAX_LINES);
	if (end_idx) {
		tail = min_t(u32, end_idx, 0x100);
		start = end_idx - tail;
		print_hex_dump(KERN_INFO, "tcm-log-raw: ",
			       DUMP_PREFIX_OFFSET, 16, 4, ring + start, tail,
			       false);
	}
}

static void sensorhub_probe_dump_iomem(const char *prefix, void __iomem *src,
				       u32 len)
{
	u8 buf[0x100];

	if (len > sizeof(buf))
		len = sizeof(buf);
	memcpy_fromio(buf, src, len);
	print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_OFFSET, 16, 4, buf, len,
		       false);
}

/*
 * The firmware writes the share-memory layout it actually uses back into
 * the carveout table header region once it has processed the first config
 * (or booted after a reset).  Read that table - carveout base + header at
 * +0x40, entries at the header's tbl_ofs - and place our rings exactly
 * where the firmware expects them.
 */
static void sensorhub_probe_read_fw_table(void)
{
	phys_addr_t share_base = scp_get_reserve_mem_phys(0);
	void __iomem *share = (void __iomem *)
		scp_get_reserve_mem_virt(SCP_A_SECDUMP_MEM_ID);
	u32 hdr_base, total, tbl_ofs, off, size, i;
	int idx, found = 0;

	if (!share || !share_base)
		return;
	hdr_base = readl(share + FW_REGION_HDR_OFS + 0x4);
	total = readl(share + FW_REGION_HDR_OFS + 0x8);
	tbl_ofs = readl(share + FW_REGION_HDR_OFS + 0xc);
	pr_info("fw-tbl: hdr base 0x%x total 0x%x tbl_ofs 0x%x (share 0x%llx)\n",
		hdr_base, total, tbl_ofs, (u64)share_base);
	if (hdr_base != (u32)share_base || tbl_ofs < FW_REGION_HDR_OFS + 0x10 ||
	    tbl_ofs > 0x1000 || total < tbl_ofs) {
		pr_info("fw-tbl: published AP layout (tbl_ofs 0x%x)\n", tbl_ofs);
		return;
	}
	if (debug) {
		sensorhub_probe_dump_iomem("fw-tbl-hdr: ",
					   share + FW_REGION_HDR_OFS, 0x40);
		sensorhub_probe_dump_iomem("fw-tbl-raw: ", share + tbl_ofs,
					   0x40);
	}
	for (i = 0; i < FW_REGION_ENTRIES; i++) {
		off = readl(share + tbl_ofs + 4 + i * 8);
		size = readl(share + tbl_ofs + 8 + i * 8);
		if (off > total || size > total - off) {
			pr_warn("fw-tbl [%u]: bad 0x%x/0x%x\n", i, off, size);
			continue;
		}
		if (debug)
			pr_info("fw-tbl [%u]: off 0x%x size 0x%x\n",
				i, off, size);
		for (idx = 0; idx < SHM_NR; idx++)
			if (shm_ids[idx] == i && off && size) {
				shm_fw_off[idx] = off;
				shm_fw_size[idx] = size;
				found++;
			}
	}
	for (idx = 0; idx < SHM_NR; idx++)
		if (shm_fw_off[idx])
			pr_info("fw-tbl idx %d id %u: off 0x%x size 0x%x\n",
				idx, shm_ids[idx], shm_fw_off[idx],
				shm_fw_size[idx]);
	shm_fw_tbl_ok = found > 0;
}

static void sensorhub_probe_dump_fwlog(const char *why, bool force)
{
	u8 __iomem *logmem;
	u32 logsize;
	bool awake;

	if (!debug && !force)
		return;

	pr_info("---- fwlog dump (%s) begin ----\n", why);

	/*
	 * TCM is only accessible while the SCP core is awake; reading it when
	 * the firmware has entered its low-power state raised an SError on
	 * the AP.  Same protection the vendor logger read uses.  In the STOP
	 * path (reset in progress) the firmware state registers must not be
	 * touched - only the already-published log ring is safe.
	 */
	awake = scp_awake_lock((void *)SCP_A_ID) == 0;
	if (!awake) {
		pr_info("tcm-log (%s): SCP not awake, skipping TCM state\n",
			why);
	} else {
		sensorhub_probe_dump_tcm_ring();
		scp_awake_unlock((void *)SCP_A_ID);
	}

	logmem = (u8 __iomem *)scp_get_reserve_mem_virt(SCP_A_LOGGER_MEM_ID);
	logsize = scp_get_reserve_mem_size(SCP_A_LOGGER_MEM_ID);
	if (logmem && logsize) {
		struct logger_log_ctrl *ctl = (struct logger_log_ctrl *)logmem;
		struct logger_buffer_info *bi;

		pr_info("dram-ctl: base 0x%x enable %u size %u info_ofs %u buff_ofs %u buff_size %u\n",
			ctl->base, ctl->enable, ctl->size, ctl->info_ofs,
			ctl->buff_ofs, ctl->buff_size);
		if (ctl->info_ofs + sizeof(*bi) <= logsize &&
		    ctl->info_ofs + sizeof(*bi) <= ctl->buff_ofs) {
			bi = (struct logger_buffer_info *)
				((u8 *)logmem + ctl->info_ofs);
			pr_info("dram-buf: r_pos %u w_pos %u\n",
				bi->r_pos, bi->w_pos);
		}
		sensorhub_probe_dump_strings("log", logmem,
					     min_t(u32, logsize, 0x40000),
					     FWLOG_MAX_LINES);
	}

	pr_info("---- fwlog dump (%s) end ----\n", why);
}

static void sensorhub_probe_fwlog_dump_work_fn(struct work_struct *work)
{
	sensorhub_probe_dump_fwlog("post-enable", false);
}

static int sensorhub_probe_logger_handler(unsigned int id, void *prdata,
		void *data, unsigned int len)
{
	struct scp_logger_ctrl_msg msg;

	if (len < sizeof(msg))
		return 0;
	memcpy(&msg, data, sizeof(msg));
	if (msg.cmd != SCP_LOGGER_IPI_INIT)
		return 0;

	logger_info = msg.u.info;
	logger_info_valid = true;
	pr_info("logger info: dram 0x%x buf 0x%x start 0x%x end 0x%x maxlen %u\n",
		logger_info.dram_addr, logger_info.buf_addr,
		logger_info.start_addr, logger_info.end_addr,
		logger_info.buf_maxlen);

	if (tcm_map && IS_ALIGNED(logger_info.dram_addr, 4) &&
	    logger_info.dram_addr <= SCP_TCM_SIZE - 4) {
		/*
		 * v24 proved that publishing the logger dram address into
		 * this TCM slot makes the firmware assert (mt_logger.c:156)
		 * within a few ms, even with the DRAM control block already
		 * enabled.  v21 (slot untouched) kept the firmware alive
		 * through the whole handshake, so inspect only -- never
		 * write here.  The firmware log is read from its TCM ring.
		 *
		 * The address comes from firmware; keep the read inside the
		 * mapping and 4-byte aligned (an unaligned device read faults).
		 */
		pr_info("logger tcm slot 0x%x = 0x%x (untouched)\n",
			logger_info.dram_addr,
			readl(tcm_map + logger_info.dram_addr));
	}
	schedule_delayed_work(&fwlog_dump_work, msecs_to_jiffies(80));
	queue_work(probe_wq, &logger_notify_work);
	return 0;
}

static int sensorhub_probe_logger_send_init(void)
{
	struct scp_logger_ctrl_msg msg;
	int ret = -EINVAL, i;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = SCP_LOGGER_IPI_INIT;
	msg.u.init.addr = (u32)scp_get_reserve_mem_phys(SCP_A_LOGGER_MEM_ID);
	msg.u.init.size = scp_get_reserve_mem_size(SCP_A_LOGGER_MEM_ID);
	for (i = 0; i < 100; i++) {
		ret = mtk_ipi_send(&scp_ipidev, IPI_OUT_LOGGER_CTRL, 0, &msg,
				   DIV_ROUND_UP(sizeof(msg), MBOX_SLOT_SIZE),
				   0);
		if (ret == IPI_ACTION_DONE)
			break;
		usleep_range(2000, 3000);
	}
	return ret;
}

static void sensorhub_probe_logger_notify_work_fn(struct work_struct *work)
{
	struct logger_log_ctrl *ctl;
	int ret;

	ctl = (struct logger_log_ctrl *)
		scp_get_reserve_mem_virt(SCP_A_LOGGER_MEM_ID);
	ret = sensorhub_probe_logger_send_init();
	pr_info("logger init ipi %s (%d)\n",
		ret == IPI_ACTION_DONE ? "sent" : "failed", ret);
	/*
	 * Vendor flow: the enable flag is raised right after the INIT IPI
	 * (or right away when the IPI fails); the firmware's logger task
	 * expects it within its poll window once the dram slot is written.
	 */
	if (ctl)
		ctl->enable = 1;
	dsb(sy);
}

static void sensorhub_probe_logger_prepare(void)
{
	struct logger_log_ctrl *ctl;
	struct logger_buffer_info *binfo;
	unsigned long virt;
	u32 size, ofs;

	virt = scp_get_reserve_mem_virt(SCP_A_LOGGER_MEM_ID);
	size = scp_get_reserve_mem_size(SCP_A_LOGGER_MEM_ID);
	if (!virt || !size) {
		pr_warn("logger region missing (virt %lx size %u)\n",
			virt, size);
		return;
	}

	ctl = (struct logger_log_ctrl *)virt;
	ofs = sizeof(*ctl) + sizeof(*binfo);
	if (ofs + LOGGER_DRAM_BUF_LEN > size) {
		pr_warn("logger region too small (%u bytes)\n", size);
		return;
	}

	/*
	 * The whole control block must be in DRAM before the dram slot is
	 * handed to the firmware: its timer task starts consuming it as
	 * soon as the slot becomes non-zero, and it must find enable set
	 * (see sensorhub_probe_logger_handler()).
	 */
	ctl->base = SCP_LOGGER_MAGIC;
	ctl->size = sizeof(*ctl);
	ctl->enable = 0;
	ctl->info_ofs = sizeof(*ctl);
	ctl->buff_ofs = ofs;
	ctl->buff_size = LOGGER_DRAM_BUF_LEN;
	binfo = (struct logger_buffer_info *)((u8 *)virt + ctl->info_ofs);
	binfo->r_pos = 0;
	binfo->w_pos = 0;
	dsb(sy);
	pr_info("logger prepared: virt %lx size %u (buff_ofs %u)\n",
		virt, size, ofs);
}

static int sensorhub_probe_ctrl_handler(unsigned int id, void *prdata,
		void *data, unsigned int len)
{
	struct sensor_comm_ack *ack = data;
	u8 crc;

	atomic_inc(&ctrl_acks);
	if (len >= offsetof(typeof(*ack), crc8) + 1) {
		crc = tiny_crc8((u8 *)ack, offsetof(typeof(*ack), crc8));
		last_ack_ret = ack->ret_val;
		pr_info("ctrl ack seq=%u type=%u cmd=%u ret=%d crc=%u/%u%s\n",
			ack->sequence, ack->sensor_type, ack->command,
			ack->ret_val, ack->crc8, crc,
			ack->crc8 == crc ? "" : " BAD");
	} else {
		pr_info("ctrl ack ipi=%u len=%u\n", id, len);
		print_hex_dump(KERN_INFO, "ctrl ack: ", DUMP_PREFIX_OFFSET,
			       16, 1, data,
			       min_t(unsigned int, len, sizeof(ctrl_rx)),
			       false);
	}
	return 0;
}

static void sensorhub_probe_drain_data(u32 wp)
{
	struct share_mem_data sample;
	u32 rp = data_rp;
	u32 item = sizeof(sample);
	u32 bs = shm_bufsize[0];
	u8 __iomem *area = shm_area[0];
	int guard = 0;

	if (!area || !bs || wp == rp)
		return;

	if (wp >= bs || wp % item) {
		if (!data_wp_bogus) {
			data_wp_bogus = true;
			pr_warn("bogus write_position %u (buffer %u item %u)\n",
				wp, bs, item);
		}
		return;
	}

	while (rp != wp && guard++ < 256) {
		u32 off = rp;
		u32 tail = bs - off;
		int cnt = atomic_inc_return(&data_samples);

		if (tail >= item) {
			memcpy_fromio(&sample, area + off, item);
		} else {
			memcpy_fromio(&sample, area + off, tail);
			memcpy_fromio((u8 *)&sample + tail, area, item - tail);
		}
		rp += item;
		if (rp >= bs)
			rp -= bs;

		if (!sample.action) {
			int axis;

			if (sample.sensor_type == SENSOR_TYPE_ACCELEROMETER) {
				for (axis = 0; axis < 3; axis++)
					sensorhub_last[axis] = sample.value[axis];
				sensorhub_probe_iio_push(&sample);
			} else if (sample.sensor_type == SENSOR_TYPE_GYROSCOPE) {
				for (axis = 0; axis < 3; axis++)
					sensorhub_gyro_last[axis] = sample.value[axis];
				sensorhub_probe_gyro_push(&sample);
				gyro_samples++;
				if (gyro_samples <= 3 ||
				    (debug && !(gyro_samples % 200)))
					pr_info("gyro #%d x=%d y=%d z=%d ts=%lld\n",
						gyro_samples, sample.value[0],
						sample.value[1], sample.value[2],
						sample.timestamp);
			} else if (sample.sensor_type == SENSOR_TYPE_MAGNETIC_FIELD) {
				for (axis = 0; axis < 3; axis++)
					sensorhub_mag_last[axis] = sample.value[axis];
				sensorhub_probe_push_s32(sensorhub_mag_iio,
							 sample.value, 3,
							 sample.timestamp);
				mag_samples++;
				if (mag_samples <= 3 ||
				    (debug && !(mag_samples % 200)))
					pr_info("mag #%d x=%d y=%d z=%d ts=%lld\n",
						mag_samples, sample.value[0],
						sample.value[1], sample.value[2],
						sample.timestamp);
			} else if (sample.sensor_type == SENSOR_TYPE_LIGHT) {
				sensorhub_light_last = sample.value[0];
				sensorhub_probe_push_s32(sensorhub_light_iio,
							 sample.value, 1,
							 sample.timestamp);
				light_samples++;
				if (light_samples <= 3 ||
				    (debug && !(light_samples % 200)))
					pr_info("light #%d value=%d ts=%lld\n",
						light_samples, sample.value[0],
						sample.timestamp);
			} else if (sample.sensor_type == SENSOR_TYPE_CCT ||
				   sample.sensor_type == SENSOR_TYPE_LUX_AOD) {
				s64 now = ktime_get_boottime_ns();
				s64 *last = sample.sensor_type == SENSOR_TYPE_CCT ?
					    &cct_diag_ts : &lux_diag_ts;

				if (now - *last > 1000000000LL) {
					*last = now;
					pr_info("diag type=%u v=%d,%d,%d,%d,%d,%d ts=%lld\n",
						sample.sensor_type,
						sample.value[0], sample.value[1],
						sample.value[2], sample.value[3],
						sample.value[4], sample.value[5],
						sample.timestamp);
				}
			} else if (sample.sensor_type == SENSOR_TYPE_PROXIMITY) {
				sensorhub_prox_last = sample.value[0];
				sensorhub_probe_push_s32(sensorhub_prox_iio,
							 sample.value, 1,
							 sample.timestamp);
				prox_samples++;
				if (prox_samples <= 3 ||
				    (debug && !(prox_samples % 200)))
					pr_info("prox #%d value=%d ts=%lld\n",
						prox_samples, sample.value[0],
						sample.timestamp);
			}
		}
		if (cnt <= 3 || (debug && !(cnt % 200)))
			pr_info("sample #%d type=%u action=%u acc=%u x=%d y=%d z=%d ts=%lld\n",
				cnt, sample.sensor_type, sample.action,
				sample.accurancy, sample.value[0],
				sample.value[1], sample.value[2],
				sample.timestamp);
	}
	data_rp = rp;
	writel(rp, shm_base[0]);
}

static void sensorhub_probe_drain_super(u32 wp)
{
	struct share_mem_super_data sample;
	u32 rp = super_rp;
	u32 item = sizeof(sample);
	u32 bs = shm_bufsize[1];
	u8 __iomem *area = shm_area[1];
	int guard = 0;

	if (!area || !bs || wp == rp)
		return;
	if (wp >= bs || wp % item) {
		pr_warn_ratelimited("super bogus write_position %u\n", wp);
		return;
	}
	while (rp != wp && guard++ < 256) {
		u32 off = rp;
		u32 tail = bs - off;

		if (tail >= item)
			memcpy_fromio(&sample, area + off, item);
		else {
			memcpy_fromio(&sample, area + off, tail);
			memcpy_fromio((u8 *)&sample + tail, area, item - tail);
		}
		rp += item;
		if (rp >= bs)
			rp -= bs;

		if (sample.action)
			continue;
		if (sample.sensor_type == SENSOR_TYPE_CCT) {
			s64 now = ktime_get_boottime_ns();

			sensorhub_light_last = sample.value[10];
			sensorhub_probe_push_s32(sensorhub_light_iio,
						 &sample.value[10], 1,
						 sample.timestamp);
			if (now - cct_diag_ts > 1000000000LL) {
				cct_diag_ts = now;
				pr_info("cct lux=%d cct=%dK rgc=%d/%d/%d/%d ts=%lld\n",
					sample.value[10], sample.value[0],
					sample.value[3], sample.value[4],
					sample.value[5], sample.value[6],
					sample.timestamp);
				if (debug)
					pr_info("super cct v=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
						sample.value[0], sample.value[1],
						sample.value[2], sample.value[3],
						sample.value[4], sample.value[5],
						sample.value[6], sample.value[7],
						sample.value[8], sample.value[9],
						sample.value[10], sample.value[11],
						sample.value[12], sample.value[13],
						sample.value[14], sample.value[15]);
			}
		} else {
			s64 now = ktime_get_boottime_ns();

			if (now - lux_diag_ts > 1000000000LL) {
				lux_diag_ts = now;
				pr_info("super type=%u v=%d,%d,%d,%d ts=%lld\n",
					sample.sensor_type, sample.value[0],
					sample.value[1], sample.value[2],
					sample.value[3], sample.timestamp);
			}
		}
	}
	super_rp = rp;
}

static void sensorhub_probe_dump_sensor_list(s32 wp)
{
	struct share_mem_info info;
	u8 __iomem *area = shm_area[2];
	u32 bs = shm_bufsize[2];
	u32 item = sizeof(info);
	u32 n, i;

	if (!area || !bs || wp <= 0 || (u32)wp > bs)
		return;
	n = (u32)wp / item;
	pr_info("sensor list: %u bytes, %u entries\n", (u32)wp, n);
	for (i = 0; i < n && i < 64; i++) {
		memcpy_fromio(&info, area + i * item, item);
		pr_info("sensor[%u] type=%u gain=0x%08x name='%.16s' vendor='%.16s'\n",
			i, info.sensor_type, info.gain,
			info.name, info.vendor);
		if (info.sensor_type == SENSOR_TYPE_GYROSCOPE && info.gain) {
			gyro_scale_micro = 1000000 / info.gain;
			pr_info("gyro scale set to %u urad/s per LSB (gain %u)\n",
				gyro_scale_micro, info.gain);
		}
	}
}

static int sensorhub_probe_notify_handler(unsigned int id, void *prdata,
		void *data, unsigned int len)
{
	struct sensor_comm_notify *n = data;
	u8 crc;

	atomic_inc(&notify_count);
	if (len <= offsetof(typeof(*n), crc8)) {
		pr_warn("short notify len=%u\n", len);
		return 0;
	}
	crc = tiny_crc8((u8 *)n, offsetof(typeof(*n), crc8));

	if (n->command == SENS_COMM_NOTIFY_DATA_CMD) {
		struct data_notify *dn = (struct data_notify *)n->value;
		int cnt = atomic_inc_return(&data_notify_count);

		if (len >= offsetof(typeof(*n), value) + sizeof(*dn))
			sensorhub_probe_drain_data((u32)dn->write_position);
		if (cnt <= 3 || (debug && !(cnt % 200)))
			pr_info("DATA notify #%d wp=%d ts=%lld crc=%s\n",
				cnt, dn->write_position, dn->scp_timestamp,
				n->crc8 == crc ? "ok" : "BAD");
		return 0;
	}

	if (n->command == SENS_COMM_NOTIFY_SUPER_DATA_CMD ||
	    n->command == SENS_COMM_NOTIFY_SUPER_FULL_CMD) {
		struct data_notify *dn = (struct data_notify *)n->value;
		int cnt = atomic_inc_return(&super_notify_count);

		if (len >= offsetof(typeof(*n), value) + sizeof(*dn))
			sensorhub_probe_drain_super((u32)dn->write_position);
		if (cnt <= 3 || (debug && !(cnt % 200)))
			pr_info("SUPER notify #%d wp=%d ts=%lld crc=%s\n",
				cnt, dn->write_position, dn->scp_timestamp,
				n->crc8 == crc ? "ok" : "BAD");
		return 0;
	}

	if (n->command == SENS_COMM_NOTIFY_LIST_CMD) {
		pr_info("sensor list reply: wp=%d crc=%s\n", n->value[0],
			n->crc8 == crc ? "ok" : "BAD");
		if (n->crc8 == crc)
			sensorhub_probe_dump_sensor_list(n->value[0]);
		return 0;
	}

	pr_info("notify seq=%u type=%u cmd=%u len=%u crc=%u/%u%s\n",
		n->sequence, n->sensor_type, n->command, n->length,
		n->crc8, crc, n->crc8 == crc ? "" : " BAD");
	print_hex_dump(KERN_INFO, "notify: ", DUMP_PREFIX_OFFSET, 16, 1,
		       data, min_t(unsigned int, len, sizeof(notify_rx)), false);

	if (n->command == SENS_COMM_NOTIFY_READY_CMD && !sensor_ready) {
		sensor_ready = true;
		pr_info("SCP sensorhub reported READY\n");
		mod_delayed_work(probe_wq, &probe_work, msecs_to_jiffies(500));
	}
	return 0;
}

static int sensorhub_probe_send_ready(void)
{
	struct sensor_comm_notify notify = {};
	int ret;

	mutex_lock(&tx_lock);
	notify.sequence = tx_sequence++;
	notify.sensor_type = SENSOR_TYPE_INVALID;
	notify.command = SENS_COMM_NOTIFY_READY_CMD;
	notify.length = 0;
	notify.crc8 = tiny_crc8((u8 *)&notify, offsetof(typeof(notify), crc8));
	ret = mtk_ipi_send(&scp_ipidev, IPI_OUT_SENSOR_NOTIFY, 0, &notify,
			   DIV_ROUND_UP(sizeof(notify), MBOX_SLOT_SIZE), 0);
	mutex_unlock(&tx_lock);
	if (ret < 0)
		pr_warn("send ready frame failed %d\n", ret);
	else
		pr_info("ready frame sent (seq=%u)\n", notify.sequence);
	return ret;
}

static int sensorhub_probe_request_sensor_list(void)
{
	struct sensor_comm_notify notify = {};
	int ret;

	if (!shm_base[2] || !shm_bufsize[2]) {
		pr_info("sensor list ring not announced\n");
		return -ENODEV;
	}
	writel(0, shm_base[2]);
	mutex_lock(&tx_lock);
	notify.sequence = tx_sequence++;
	notify.sensor_type = SENSOR_TYPE_INVALID;
	notify.command = SENS_COMM_NOTIFY_LIST_CMD;
	notify.length = 0;
	notify.crc8 = tiny_crc8((u8 *)&notify, offsetof(typeof(notify), crc8));
	ret = mtk_ipi_send(&scp_ipidev, IPI_OUT_SENSOR_NOTIFY, 0, &notify,
			   DIV_ROUND_UP(sizeof(notify), MBOX_SLOT_SIZE), 0);
	mutex_unlock(&tx_lock);
	if (ret < 0)
		pr_warn("sensor list request failed %d\n", ret);
	else
		pr_info("sensor list request sent (seq=%u)\n", notify.sequence);
	return ret;
}

/*
 * Send one control frame and wait 100 ms for the ack.  Returns 1 if an
 * ack arrived, 0 if none, negated errno if the IPI send itself failed.
 */
static int sensorhub_probe_ctrl_xfer(u8 sensor_type, u8 command,
		const void *payload, u8 payload_len)
{
	struct sensor_comm_ctrl *ctrl;
	size_t size = sizeof(*ctrl) + payload_len;
	int ret = -EINVAL, i, acks_before;

	ctrl = kzalloc(size, GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	mutex_lock(&tx_lock);
	ctrl->sequence = tx_sequence++;
	ctrl->sensor_type = sensor_type;
	ctrl->command = command;
	ctrl->length = payload_len;
	if (payload_len)
		memcpy(ctrl->data, payload, payload_len);
	ctrl->crc8 = tiny_crc8((u8 *)ctrl, offsetof(typeof(*ctrl), crc8));
	last_ack_ret = -1000;
	acks_before = atomic_read(&ctrl_acks);
	for (i = 0; i < 3; i++) {
		ret = mtk_ipi_send(&scp_ipidev, IPI_OUT_SENSOR_CTRL, 0, ctrl,
				   DIV_ROUND_UP(size, MBOX_SLOT_SIZE), 0);
		if (ret >= 0)
			break;
		pr_warn("ctrl cmd %u send failed %d (try %d)\n", command, ret, i);
		usleep_range(1000, 2000);
	}
	mutex_unlock(&tx_lock);
	kfree(ctrl);
	if (ret < 0)
		return ret;

	msleep(250);
	if (atomic_read(&ctrl_acks) > acks_before)
		return 1;
	pr_warn("ctrl cmd %u (type %u): no ack within 250 ms\n",
		command, sensor_type);
	return 0;
}

static int sensorhub_probe_shm_setup(void)
{
	int i;

	for (i = 0; i < SHM_NR; i++) {
		unsigned long virt = scp_get_reserve_mem_virt(shm_ids[i]);
		phys_addr_t phys = scp_get_reserve_mem_phys(shm_ids[i]);
		u32 size = scp_get_reserve_mem_size(shm_ids[i]);
		u32 buf;
		u8 __iomem *b;

		if (shm_fw_tbl_ok && shm_fw_off[i] && shm_fw_size[i]) {
			phys = scp_get_reserve_mem_phys(0) + shm_fw_off[i];
			virt = scp_get_reserve_mem_virt(0) + shm_fw_off[i];
			size = shm_fw_size[i];
		}
		shm_phys[i] = phys;
		pr_info("region idx %d id %u type %u: phys 0x%llx virt %lx size %u item %u%s%s\n",
			i, shm_ids[i], shm_payload_types[i],
			(u64)phys, virt, size, shm_item_sizes[i],
			(shm_fw_off[i] && shm_fw_tbl_ok) ? " [fw-tbl]" : " [ap-tbl]",
			(shm_mask & BIT(i)) ? " [announce]" : "");

		if (!virt || size <= sizeof(struct share_mem_base)) {
			pr_warn("share region id %u missing (virt %lx size %u)\n",
				shm_ids[i], virt, size);
			if (shm_mask & BIT(i))
				return -ENODEV;
			continue;
		}
		/*
		 * Every ring the firmware knows about needs a valid header
		 * (rp/wp/buffer_size/item_size): its tasks touch them even
		 * when the payload type is not announced by the AP.  Only
		 * the announced rings are later wired into the data path.
		 */
		buf = (size - sizeof(struct share_mem_base)) /
			shm_item_sizes[i] * shm_item_sizes[i];
		b = (u8 __iomem *)virt;
		writel(0, b + offsetof(struct share_mem_base, rp));
		writel(0, b + offsetof(struct share_mem_base, wp));
		writel(buf, b + offsetof(struct share_mem_base, buffer_size));
		writel(shm_item_sizes[i],
		       b + offsetof(struct share_mem_base, item_size));
		if (shm_mask & BIT(i)) {
			shm_base[i] = b;
			shm_area[i] = b + offsetof(struct share_mem_base, data);
			shm_bufsize[i] = buf;
		}
		pr_info("ring idx %d %s: buffer %u bytes, item %u\n", i,
			(shm_mask & BIT(i)) ? "ready" : "headers only",
			buf, shm_item_sizes[i]);
	}
	data_rp = 0;
	data_wp_bogus = false;
	return 0;
}

static int sensorhub_probe_send_share_mem_batch(struct sensor_comm_share_mem *cfg,
						int regions)
{
	int ret;

	if (debug)
		print_hex_dump(KERN_INFO, "share-mem frame: ",
			       DUMP_PREFIX_OFFSET, 16, 1, cfg, sizeof(*cfg),
			       false);
	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_INVALID,
					SENS_COMM_CTRL_SHARE_MEMORY_CMD,
					cfg, sizeof(*cfg));
	if (ret < 0) {
		pr_warn("share-mem batch (%d regions) send failed %d\n",
			regions, ret);
		return ret;
	}
	if (ret == 0)
		pr_warn("share-mem batch (%d regions): no ack\n", regions);
	else
		pr_info("share-mem batch: %d regions acked (ret=%d)\n",
			regions, last_ack_ret);
	return 0;
}

static int sensorhub_probe_send_share_mem(void)
{
	struct sensor_comm_share_mem cfg;
	int i, idx = 0, nr = 0, ret;

	memset(&cfg, 0, sizeof(cfg));
	for (i = 0; i < SHM_NR; i++) {
		if (!(shm_mask & BIT(i)))
			continue;
		cfg.base_info[idx].payload_type = shm_payload_types[i];
		cfg.base_info[idx].payload_base = (u32)shm_phys[i];
		cfg.available_num = ++idx;
		nr++;
		if (idx < ARRAY_SIZE(cfg.base_info))
			continue;
		ret = sensorhub_probe_send_share_mem_batch(&cfg, idx);
		if (ret < 0)
			return ret;
		memset(&cfg, 0, sizeof(cfg));
		idx = 0;
	}
	if (idx) {
		ret = sensorhub_probe_send_share_mem_batch(&cfg, idx);
		if (ret < 0)
			return ret;
	}
	pr_info("share-mem config sent (%d regions)\n", nr);
	return 0;
}

static void sensorhub_probe_phase2_fn(struct work_struct *work)
{
	struct sensor_comm_timesync ts;
	struct sensor_comm_batch batch;
	struct timespec64 real;
	struct rtc_time tm;
	int ret;

	pr_info("starting accelerometer (timesync + enable)\n");
	ts.host_timestamp = ktime_get_boottime_ns();
	ts.host_archcounter = __arch_counter_get_cntvct();
	ts.sched_clock = sched_clock();
	ktime_get_real_ts64(&real);
	rtc_time64_to_tm(real.tv_sec, &tm);
	ts.usecond = real.tv_nsec / 1000;
	ts.second = tm.tm_sec;
	ts.minute = tm.tm_min;
	ts.hour = tm.tm_hour;
	ts.day = tm.tm_mday;
	ts.month = tm.tm_mon + 1;

	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_INVALID,
					SENS_COMM_CTRL_TIMESYNC_CMD,
					&ts, sizeof(ts));
	if (ret != 1)
		pr_warn("timesync not acked (%d)\n", ret);

	batch.delay = 20000000;
	batch.latency = 20000000;
	atomic_set(&data_notify_count, 0);
	atomic_set(&data_samples, 0);
	iio_ts_offset_valid = false;

	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_ACCELEROMETER,
					SENS_COMM_CTRL_ENABLE_CMD,
					&batch, sizeof(batch));
	if (ret == 1)
		pr_info("accel enabled (ret=%d)%s\n", last_ack_ret,
			test_window_s ? " for test window" : ", streaming");
	else
		pr_warn("accel enable not acked (%d)\n", ret);

	batch.delay = 10000000;
	batch.latency = 0;
	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_GYROSCOPE,
					SENS_COMM_CTRL_ENABLE_CMD,
					&batch, sizeof(batch));
	if (ret == 1)
		pr_info("gyro enabled (ret=%d), streaming\n", last_ack_ret);
	else
		pr_warn("gyro enable not acked (%d)\n", ret);

	batch.delay = 20000000;
	batch.latency = 20000000;
	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_MAGNETIC_FIELD,
					SENS_COMM_CTRL_ENABLE_CMD,
					&batch, sizeof(batch));
	if (ret == 1)
		pr_info("mag enabled (ret=%d), streaming\n", last_ack_ret);
	else
		pr_warn("mag enable not acked (%d)\n", ret);

	batch.delay = 200000000;
	batch.latency = 200000000;
	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_PROXIMITY,
					SENS_COMM_CTRL_ENABLE_CMD,
					&batch, sizeof(batch));
	if (ret == 1)
		pr_info("prox enabled (ret=%d), streaming\n", last_ack_ret);
	else
		pr_warn("prox enable not acked (%d)\n", ret);

	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_CCT,
					SENS_COMM_CTRL_ENABLE_CMD,
					&batch, sizeof(batch));
	if (ret == 1)
		pr_info("cct enabled (ret=%d), streaming\n", last_ack_ret);
	else
		pr_warn("cct enable not acked (%d)\n", ret);

	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_LUX_AOD,
					SENS_COMM_CTRL_ENABLE_CMD,
					&batch, sizeof(batch));
	if (ret == 1)
		pr_info("lux_aod enabled (ret=%d), streaming\n", last_ack_ret);
	else
		pr_warn("lux_aod enable not acked (%d)\n", ret);

	if (test_window_s)
		queue_delayed_work(probe_wq, &disable_accel_work,
				   msecs_to_jiffies(test_window_s * 1000));
}

static void sensorhub_probe_work_fn(struct work_struct *work)
{
	int ret;

	pr_info("share-mem handshake (platform_ready=%u sensor_ready=%u)\n",
		platform_ready, sensor_ready);

	sensorhub_probe_dump_fwlog("pre-config", false);
	sensorhub_probe_read_fw_table();

	if (sensorhub_probe_shm_setup() < 0)
		return;
	ret = sensorhub_probe_send_share_mem();
	if (ret < 0) {
		pr_warn("share-mem config failed %d\n", ret);
		sensorhub_probe_dump_fwlog("share-mem-failed", false);
		return;
	}
	sensorhub_probe_request_sensor_list();
	queue_delayed_work(probe_wq, &phase2_work, msecs_to_jiffies(2000));
}

static void sensorhub_probe_disable_work_fn(struct work_struct *work)
{
	int ret;

	pr_info("test window over (notifies=%d data=%d samples=%d), disabling accel\n",
		atomic_read(&notify_count), atomic_read(&data_notify_count),
		atomic_read(&data_samples));
	ret = sensorhub_probe_ctrl_xfer(SENSOR_TYPE_ACCELEROMETER,
					SENS_COMM_CTRL_DISABLE_CMD, NULL, 0);
	if (ret == 1)
		pr_info("accel disable acked (ret=%d)\n", last_ack_ret);
}

static const struct iio_mount_matrix sensorhub_probe_mount_matrix = {
	.rotation = { "-1", "0", "0", "0", "-1", "0", "0", "0", "1" },
};

static const struct iio_mount_matrix *
sensorhub_probe_get_mount_matrix(const struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	return &sensorhub_probe_mount_matrix;
}

static const struct iio_chan_spec_ext_info sensorhub_probe_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, sensorhub_probe_get_mount_matrix),
	{ },
};

#define SENSORHUB_ACCEL_CHANNEL(_axis, _scan_index) {		\
	.type = IIO_ACCEL,					\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = _scan_index,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
	.ext_info = sensorhub_probe_ext_info,			\
}

static const struct iio_chan_spec sensorhub_probe_iio_channels[] = {
	SENSORHUB_ACCEL_CHANNEL(X, 0),
	SENSORHUB_ACCEL_CHANNEL(Y, 1),
	SENSORHUB_ACCEL_CHANNEL(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

#define SENSORHUB_GYRO_CHANNEL(_axis, _scan_index) {		\
	.type = IIO_ANGL_VEL,					\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = _scan_index,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
	.ext_info = sensorhub_probe_ext_info,			\
}

static const struct iio_chan_spec sensorhub_probe_gyro_channels[] = {
	SENSORHUB_GYRO_CHANNEL(X, 0),
	SENSORHUB_GYRO_CHANNEL(Y, 1),
	SENSORHUB_GYRO_CHANNEL(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

#define SENSORHUB_MAG_CHANNEL(_axis, _scan_index) {		\
	.type = IIO_MAGN,					\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = _scan_index,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
	.ext_info = sensorhub_probe_ext_info,			\
}

static const struct iio_chan_spec sensorhub_probe_mag_channels[] = {
	SENSORHUB_MAG_CHANNEL(X, 0),
	SENSORHUB_MAG_CHANNEL(Y, 1),
	SENSORHUB_MAG_CHANNEL(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static ssize_t sensorhub_probe_prox_nearlevel(struct iio_dev *indio_dev,
					      uintptr_t private,
					      const struct iio_chan_spec *chan,
					      char *buf)
{
	return sysfs_emit(buf, "1\n");
}

static const struct iio_chan_spec_ext_info sensorhub_probe_prox_ext_info[] = {
	{
		.name = "nearlevel",
		.shared = IIO_SEPARATE,
		.read = sensorhub_probe_prox_nearlevel,
	},
	{ },
};

#define SENSORHUB_SCALAR_CHANNEL(_type) {			\
	.type = _type,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = 0,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	},							\
}

static const struct iio_chan_spec sensorhub_probe_light_channels[] = {
	SENSORHUB_SCALAR_CHANNEL(IIO_LIGHT),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec sensorhub_probe_prox_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_LE,
		},
		.ext_info = sensorhub_probe_prox_ext_info,
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static int sensorhub_probe_iio_read_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = sensorhub_last[chan->channel2 - IIO_MOD_X];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 9806;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static const struct iio_info sensorhub_probe_iio_info = {
	.read_raw = sensorhub_probe_iio_read_raw,
};

static int sensorhub_probe_gyro_read_raw(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = sensorhub_gyro_last[chan->channel2 - IIO_MOD_X];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = gyro_scale_micro;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static const struct iio_info sensorhub_probe_gyro_info = {
	.read_raw = sensorhub_probe_gyro_read_raw,
};

static int sensorhub_probe_mag_read_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = sensorhub_mag_last[chan->channel2 - IIO_MOD_X];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 10;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static const struct iio_info sensorhub_probe_mag_info = {
	.read_raw = sensorhub_probe_mag_read_raw,
};

static int sensorhub_probe_scalar_read_raw(struct iio_dev *indio_dev,
					   struct iio_chan_spec const *chan,
					   int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_LIGHT)
			*val = sensorhub_light_last;
		else
			*val = sensorhub_prox_last;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static const struct iio_info sensorhub_probe_light_info = {
	.read_raw = sensorhub_probe_scalar_read_raw,
};

static const struct iio_info sensorhub_probe_prox_info = {
	.read_raw = sensorhub_probe_scalar_read_raw,
};

static void sensorhub_probe_iio_push_sample(struct iio_dev *indio,
					    const struct share_mem_data *sample)
{
	u8 buf[32] __aligned(8) = { };
	s64 ts;
	int i;

	if (!indio || !iio_buffer_enabled(indio))
		return;

	if (!iio_ts_offset_valid) {
		iio_ts_offset = ktime_get_boottime_ns() - sample->timestamp;
		iio_ts_offset_valid = true;
	}
	for (i = 0; i < 3; i++)
		put_unaligned_le16((u16)clamp_t(s32, sample->value[i],
						S16_MIN, S16_MAX),
				   buf + i * 2);
	ts = sample->timestamp + iio_ts_offset;
	iio_push_to_buffers_with_timestamp(indio, buf, ts);
}

static void sensorhub_probe_iio_push(const struct share_mem_data *sample)
{
	sensorhub_probe_iio_push_sample(sensorhub_iio, sample);
}

static void sensorhub_probe_push_s32(struct iio_dev *indio, const s32 *vals,
				     int count, s64 scp_ts)
{
	u8 buf[32] __aligned(8) = { };
	s64 ts;
	int i;

	if (!indio || !iio_buffer_enabled(indio))
		return;
	if (!iio_ts_offset_valid) {
		iio_ts_offset = ktime_get_boottime_ns() - scp_ts;
		iio_ts_offset_valid = true;
	}
	for (i = 0; i < count; i++)
		put_unaligned_le32((u32)vals[i], buf + i * 4);
	ts = scp_ts + iio_ts_offset;
	iio_push_to_buffers_with_timestamp(indio, buf, ts);
}

static void sensorhub_probe_gyro_push(const struct share_mem_data *sample)
{
	sensorhub_probe_push_s32(sensorhub_gyro_iio, sample->value, 3,
				 sample->timestamp);
}

static const struct iio_buffer_setup_ops sensorhub_probe_buffer_ops = {
};

static void sensorhub_probe_iio_remove(struct platform_device *pdev)
{
	sensorhub_iio = NULL;
	sensorhub_gyro_iio = NULL;
	sensorhub_mag_iio = NULL;
	sensorhub_light_iio = NULL;
	sensorhub_prox_iio = NULL;
}

static int sensorhub_probe_iio_probe(struct platform_device *pdev)
{
	struct iio_dev *indio;
	int ret;

	indio = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio)
		return -ENOMEM;
	indio->name = "bmi2xy";
	indio->info = &sensorhub_probe_iio_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = sensorhub_probe_iio_channels;
	indio->num_channels = ARRAY_SIZE(sensorhub_probe_iio_channels);

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio,
					  &sensorhub_probe_buffer_ops);
	if (ret)
		return ret;
	ret = devm_iio_device_register(&pdev->dev, indio);
	if (ret)
		return ret;
	sensorhub_iio = indio;
	dev_info(&pdev->dev, "bmi2xy accelerometer registered\n");

	indio = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio)
		return -ENOMEM;
	indio->name = "bmi2xy_gyro";
	indio->info = &sensorhub_probe_gyro_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = sensorhub_probe_gyro_channels;
	indio->num_channels = ARRAY_SIZE(sensorhub_probe_gyro_channels);
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio,
					  &sensorhub_probe_buffer_ops);
	if (ret)
		return ret;
	ret = devm_iio_device_register(&pdev->dev, indio);
	if (ret)
		return ret;
	sensorhub_gyro_iio = indio;
	dev_info(&pdev->dev, "bmi2xy gyroscope registered\n");

	indio = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio)
		return -ENOMEM;
	indio->name = "akm09919_mag";
	indio->info = &sensorhub_probe_mag_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = sensorhub_probe_mag_channels;
	indio->num_channels = ARRAY_SIZE(sensorhub_probe_mag_channels);
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio,
					  &sensorhub_probe_buffer_ops);
	if (ret)
		return ret;
	ret = devm_iio_device_register(&pdev->dev, indio);
	if (ret)
		return ret;
	sensorhub_mag_iio = indio;
	dev_info(&pdev->dev, "akm09919 magnetometer registered\n");

	indio = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio)
		return -ENOMEM;
	indio->name = "tcs3720_light";
	indio->info = &sensorhub_probe_light_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = sensorhub_probe_light_channels;
	indio->num_channels = ARRAY_SIZE(sensorhub_probe_light_channels);
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio,
					  &sensorhub_probe_buffer_ops);
	if (ret)
		return ret;
	ret = devm_iio_device_register(&pdev->dev, indio);
	if (ret)
		return ret;
	sensorhub_light_iio = indio;
	dev_info(&pdev->dev, "tcs3720 light registered (cct lux)\n");

	indio = devm_iio_device_alloc(&pdev->dev, 0);
	if (!indio)
		return -ENOMEM;
	indio->name = "tcs3720_ps";
	indio->info = &sensorhub_probe_prox_info;
	indio->modes = INDIO_DIRECT_MODE;
	indio->channels = sensorhub_probe_prox_channels;
	indio->num_channels = ARRAY_SIZE(sensorhub_probe_prox_channels);
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio,
					  &sensorhub_probe_buffer_ops);
	if (ret)
		return ret;
	ret = devm_iio_device_register(&pdev->dev, indio);
	if (ret)
		return ret;
	sensorhub_prox_iio = indio;
	dev_info(&pdev->dev, "tcs3720 proximity registered\n");
	return 0;
}

static struct platform_driver sensorhub_probe_iio_driver = {
	.probe = sensorhub_probe_iio_probe,
	.remove = sensorhub_probe_iio_remove,
	.driver = { .name = "sensorhub_probe" },
};

static int sensorhub_probe_scp_notifier(struct notifier_block *nb,
		unsigned long event, void *p)
{
	if (event == SCP_EVENT_READY) {
		platform_ready = true;
		pr_info("SCP platform ready (is_scp_ready=%u), handshaking\n",
			is_scp_ready(SCP_A_ID));
		sensorhub_probe_send_ready();
		queue_work(probe_wq, &logger_notify_work);
		queue_delayed_work(probe_wq, &probe_work,
				   msecs_to_jiffies(15000));
	} else if (event == SCP_EVENT_STOP) {
		platform_ready = false;
		sensor_ready = false;
		data_rp = 0;
		iio_ts_offset_valid = false;
		cancel_delayed_work(&phase2_work);
		cancel_delayed_work(&disable_accel_work);
		sensorhub_probe_dump_fwlog("scp-stop", true);
		logger_info_valid = false;
		pr_info("SCP stopped\n");
	}
	return NOTIFY_DONE;
}

static struct notifier_block sensorhub_probe_scp_nb = {
	.notifier_call = sensorhub_probe_scp_notifier,
};

static int __init sensorhub_probe_init(void)
{
	int ret;

	probe_wq = alloc_ordered_workqueue("sensorhub_probe", WQ_MEM_RECLAIM);
	if (!probe_wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&probe_work, sensorhub_probe_work_fn);
	INIT_DELAYED_WORK(&disable_accel_work, sensorhub_probe_disable_work_fn);
	INIT_WORK(&logger_notify_work, sensorhub_probe_logger_notify_work_fn);
	INIT_DELAYED_WORK(&fwlog_dump_work, sensorhub_probe_fwlog_dump_work_fn);
	INIT_DELAYED_WORK(&phase2_work, sensorhub_probe_phase2_fn);

	sensorhub_probe_map_tcm();
	sensorhub_probe_logger_prepare();

	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_SENSOR_CTRL,
			       sensorhub_probe_ctrl_handler, NULL, ctrl_rx);
	if (ret < 0) {
		pr_err("register ipi %d failed %d\n", IPI_IN_SENSOR_CTRL, ret);
		goto err_wq;
	}
	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_SENSOR_NOTIFY,
			       sensorhub_probe_notify_handler, NULL, notify_rx);
	if (ret < 0) {
		pr_err("register ipi %d failed %d\n", IPI_IN_SENSOR_NOTIFY,
		       ret);
		goto err_ctrl;
	}
	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_LOGGER_CTRL,
			       sensorhub_probe_logger_handler, NULL, logger_rx);
	if (ret < 0) {
		pr_err("register ipi %d failed %d\n", IPI_IN_LOGGER_CTRL, ret);
		goto err_notify;
	}

	ret = platform_driver_register(&sensorhub_probe_iio_driver);
	if (ret) {
		pr_err("iio driver register failed %d\n", ret);
		goto err_logger;
	}
	sensorhub_probe_iio_pdev = platform_device_register_simple(
		"sensorhub_probe", -1, NULL, 0);
	if (IS_ERR(sensorhub_probe_iio_pdev)) {
		ret = PTR_ERR(sensorhub_probe_iio_pdev);
		sensorhub_probe_iio_pdev = NULL;
		pr_err("iio device register failed %d\n", ret);
		goto err_driver;
	}

	scp_A_register_notify(&sensorhub_probe_scp_nb);
	pr_info("loaded, listening on ipi %d/%d/%d (shm_mask 0x%x)\n",
		IPI_IN_SENSOR_CTRL, IPI_IN_SENSOR_NOTIFY, IPI_IN_LOGGER_CTRL,
		shm_mask);
	return 0;

err_driver:
	platform_driver_unregister(&sensorhub_probe_iio_driver);
err_logger:
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_LOGGER_CTRL);
err_notify:
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_SENSOR_NOTIFY);
err_ctrl:
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_SENSOR_CTRL);
err_wq:
	destroy_workqueue(probe_wq);
	return ret;
}

static void __exit sensorhub_probe_exit(void)
{
	scp_A_unregister_notify(&sensorhub_probe_scp_nb);
	cancel_delayed_work_sync(&probe_work);
	cancel_delayed_work_sync(&phase2_work);
	cancel_delayed_work_sync(&disable_accel_work);
	cancel_work_sync(&logger_notify_work);
	cancel_delayed_work_sync(&fwlog_dump_work);
	platform_device_unregister(sensorhub_probe_iio_pdev);
	platform_driver_unregister(&sensorhub_probe_iio_driver);
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_SENSOR_CTRL);
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_SENSOR_NOTIFY);
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_LOGGER_CTRL);
	if (tcm_map)
		iounmap(tcm_map);
	destroy_workqueue(probe_wq);
}

module_init(sensorhub_probe_init);
module_exit(sensorhub_probe_exit);

MODULE_AUTHOR("liudaoziwa");
MODULE_DESCRIPTION("BMI2XY accelerometer via the MT6991 SCP sensorhub");
MODULE_LICENSE("GPL");
