// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6991 board early-PMIC rail bring-up.
 *
 * The mainline PMIF controller driver (spmi-mtk-pmif) cannot finish a
 * 16-bit extended register access (SPMI opc 3, PMIF_CMD_EXT_REG_LONG) on
 * MT6991: every such transaction times out waiting for SWINF_WFVLDCLR, while
 * 8-bit register accesses on the same channel work fine.  That rules out the
 * usual regmap/SPMI route for the MT6363/MT6373 rails, which is why the board
 * supplies were raised from the initramfs over /dev/mem -- long after the
 * consumers that need them had already probed, and therefore why the touch
 * controller had to be unbound/rebound to come up with its AVDD rail on.
 *
 * This driver keeps those sequences in DT but runs them from the kernel at
 * subsys_initcall, which is strictly before any device_initcall driver (the
 * SPI6 controller, the SPMI controller itself) can register -- so consumers
 * probe with their rails already up.  The SPMI access is the same bare SWINF
 * handshake the initramfs used, so nothing here depends on the buggy PMIF
 * path.
 *
 * Every sequence is a read-modify-write list straight out of DT:
 *
 *   mediatek,steps  = <reg mask value delay_us>...
 *     mask != ~0 : reg = (reg & ~mask) | (value & mask)
 *     mask == ~0 : plain write of value (no read, safe for write-only regs)
 *     delay_us    : settle time after the write
 *
 *   mediatek,state = <reg mask value>...
 *     If every entry already matches, the sequence is skipped; after it runs
 *     the same list is checked again and a mismatch is reported.  Omit it for
 *     sequences that have no resting state (reset pulses).
 *
 * A child carrying mediatek,state but no mediatek,steps is a read-only
 * sequence: the registers are read and compared, mismatches are logged, and
 * nothing is written.  Use it for "did the firmware/hardware already put this
 * where we expect" probes that must not perturb the system.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define SWINF_ACC		0x00
#define SWINF_WDATA		0x04
#define SWINF_RDATA		0x14
#define SWINF_VLD_CLR		0x24
#define SWINF_STA		0x28

#define SWINF_IDLE		0x0
#define SWINF_WFVLDCLR		0x6
#define SWINF_STATE(v)		(((v) >> 1) & 0x7)

#define SPMI_CMD_EXT_REG_LONG	3
#define SPMI_CMD_WRITE		(1u << 29)

#define SWINF_POLL_US		100
#define SWINF_POLL_ROUNDS	200

#define STEP_CELLS		4	/* reg, mask, value, delay_us */
#define STATE_CELLS		3	/* reg, mask, value */

struct pmic_early_bus {
	struct device_node *np;
	void __iomem *base;
	resource_size_t phys;
	bool spmi;
	u32 acc;			/* SWINF ACC offset inside the window */
};

struct pmic_early_seq {
	const char *name;
	struct pmic_early_bus *bus;
	u32 usid;
	u32 *steps;
	u32 nsteps;			/* in cells */
	u32 *state;
	u32 nstate;			/* in cells */
};

static int pmic_early_swinf_wait(struct pmic_early_bus *bus, u32 state)
{
	u32 v;
	int i;

	for (i = 0; i < SWINF_POLL_ROUNDS; i++) {
		v = readl(bus->base + bus->acc + SWINF_STA);
		if (SWINF_STATE(v) == state)
			return 0;
		usleep_range(SWINF_POLL_US, SWINF_POLL_US + 50);
	}
	return -ETIMEDOUT;
}

static int pmic_early_read(struct pmic_early_bus *bus, u32 usid, u32 off,
			   u32 *val)
{
	int i, ret;

	if (!bus->spmi) {
		*val = readl(bus->base + off);
		return 0;
	}
	if (off > 0xffff)
		return -EINVAL;

	writel(1, bus->base + bus->acc + SWINF_VLD_CLR);
	ret = pmic_early_swinf_wait(bus, SWINF_IDLE);
	if (ret)
		return ret;

	writel((SPMI_CMD_EXT_REG_LONG << 30) | (usid << 24) | off,
	       bus->base + bus->acc + SWINF_ACC);

	for (i = 0; i < SWINF_POLL_ROUNDS; i++) {
		u32 v = readl(bus->base + bus->acc + SWINF_STA);

		if (SWINF_STATE(v) == SWINF_WFVLDCLR) {
			*val = readl(bus->base + bus->acc + SWINF_RDATA) & 0xff;
			writel(1, bus->base + bus->acc + SWINF_VLD_CLR);
			return 0;
		}
		usleep_range(SWINF_POLL_US, SWINF_POLL_US + 50);
	}
	return -ETIMEDOUT;
}

static int pmic_early_write(struct pmic_early_bus *bus, u32 usid, u32 off,
			    u32 mask, u32 val)
{
	int ret;
	u32 cur = 0;

	if (mask != 0xffffffff) {
		ret = pmic_early_read(bus, usid, off, &cur);
		if (ret)
			return ret;
		val = (cur & ~mask) | (val & mask);
	}

	if (!bus->spmi) {
		writel(val, bus->base + off);
		return 0;
	}
	if (off > 0xffff)
		return -EINVAL;

	writel(1, bus->base + bus->acc + SWINF_VLD_CLR);
	ret = pmic_early_swinf_wait(bus, SWINF_IDLE);
	if (ret)
		return ret;

	writel(val & 0xff, bus->base + bus->acc + SWINF_WDATA);
	writel((SPMI_CMD_EXT_REG_LONG << 30) | SPMI_CMD_WRITE | (usid << 24) | off,
	       bus->base + bus->acc + SWINF_ACC);

	return pmic_early_swinf_wait(bus, SWINF_IDLE);
}

/*
 * Check every <reg mask value> triple.  Returns 0 when they all match;
 * non-zero otherwise.  Individual mismatches are only reported when the
 * caller asks, so the pre-run check does not spam the boot log with the
 * state we are about to change.
 */
static int pmic_early_check(struct device *dev, struct pmic_early_seq *seq,
			    bool report)
{
	u32 i, bad = 0;

	for (i = 0; i < seq->nstate; i += STATE_CELLS) {
		const u32 *c = &seq->state[i];
		u32 val = 0;
		int ret;

		ret = pmic_early_read(seq->bus, seq->usid, c[0], &val);
		if (ret) {
			dev_err(dev, "%s: read 0x%x failed (%d)\n",
				seq->name, c[0], ret);
			return ret;
		}
		if ((val & c[1]) == (c[2] & c[1]))
			continue;
		bad++;
		if (report)
			dev_err(dev, "%s: 0x%x = 0x%x, expected 0x%x (mask 0x%x)\n",
				seq->name, c[0], val, c[2], c[1]);
	}
	return bad ? -EIO : 0;
}

/*
 * Read-only sequences print every register they look at, whether or not it
 * matches: they exist to answer a question about the current hardware state,
 * and "it matched" is only useful if the value that matched is visible.
 */
static void pmic_early_dump(struct device *dev, struct pmic_early_seq *seq)
{
	u32 i;

	for (i = 0; i < seq->nstate; i += STATE_CELLS) {
		const u32 *c = &seq->state[i];
		u32 val = 0;
		int ret;

		ret = pmic_early_read(seq->bus, seq->usid, c[0], &val);
		if (ret)
			dev_err(dev, "%s: read 0x%x failed (%d)\n",
				seq->name, c[0], ret);
		else
			dev_info(dev, "%s: [0x%x]=0x%02x (mask 0x%x want 0x%x)\n",
				 seq->name, c[0], val, c[1], c[2]);
	}
}

static int pmic_early_run(struct device *dev, struct pmic_early_seq *seq)
{
	u32 i;
	int ret;

	if (!seq->nsteps) {
		/* Read-only sequence: report, never write, never fail probe. */
		pmic_early_dump(dev, seq);
		return 0;
	}

	if (!seq->nstate)
		dev_info(dev, "%s: running (no state to check)\n", seq->name);

	if (seq->nstate && !pmic_early_check(dev, seq, false)) {
		dev_info(dev, "%s: already in the target state\n", seq->name);
		return 0;
	}

	for (i = 0; i < seq->nsteps; i += STEP_CELLS) {
		const u32 *st = &seq->steps[i];

		ret = pmic_early_write(seq->bus, seq->usid, st[0], st[1], st[2]);
		if (ret) {
			dev_err(dev, "%s: write 0x%x failed (%d)\n",
				seq->name, st[0], ret);
			return ret;
		}
		if (st[3])
			usleep_range(st[3], st[3] + 500);
	}

	if (!seq->nstate)
		return 0;

	ret = pmic_early_check(dev, seq, true);
	if (ret)
		dev_err(dev, "%s: target state not reached\n", seq->name);
	else
		dev_info(dev, "%s: up\n", seq->name);
	return ret;
}

static int pmic_early_bus_parse(struct device *dev, struct device_node *np,
				struct pmic_early_bus *bus)
{
	struct resource res;
	int ret;

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return dev_err_probe(dev, ret, "%pOF: bad reg\n", np);

	/*
	 * Deliberately devm_ioremap() and not devm_ioremap_resource(): the
	 * SPMI controller maps the very same window through devm_of_iomap()
	 * (no request_mem_region), so claiming it here would only be a
	 * bookkeeping fight with no safety benefit.
	 */
	bus->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!bus->base)
		return dev_err_probe(dev, -ENOMEM, "%pOF: ioremap failed\n",
				     np);

	bus->np = np;
	bus->phys = res.start;
	ret = of_property_read_u32(np, "mediatek,swinf-offset", &bus->acc);
	bus->spmi = !ret;
	if (bus->spmi && bus->acc > 0xf00)
		return dev_err_probe(dev, -EINVAL,
				     "%pOF: bad mediatek,swinf-offset\n", np);

	dev_info(dev, "%pOF: window @%pa%s\n", np, &bus->phys,
		 bus->spmi ? " (SPMI/SWINF)" : "");
	return 0;
}

static int pmic_early_cells(struct device *dev, struct device_node *np,
			    const char *prop, int cellsize, u32 **out, u32 *n)
{
	u32 *arr;
	int count, ret;

	count = of_property_count_elems_of_size(np, prop, sizeof(u32));
	if (count < 0)
		return count;
	if (!count || count % cellsize)
		return dev_err_probe(dev, -EINVAL,
				     "%pOF: %s is not a multiple of %d cells\n",
				     np, prop, cellsize);

	arr = devm_kcalloc(dev, count, sizeof(u32), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, prop, arr, count);
	if (ret < 0)
		return ret;

	*out = arr;
	*n = count;
	return 0;
}

/*
 * Runtime SPMI poke, for bringing up rails that were not known when the
 * board was described.  The boot sequences above are static DT lists;
 * these two files let a developer read or write any PMIC register while
 * the system is running, using the same bare SWINF path:
 *
 *   echo "5 0x1c23 0x01" > spmi_cmd    # write 0x01 to MT6373 0x1c23
 *   echo "5 0x1c23"      > spmi_cmd    # read it into spmi_val
 *   cat spmi_val                       # "0x01 0"
 *
 * A fourth cell is a mask for read-modify-write.  Writes are deliberately
 * unrestricted: this is a bring-up aid, not a production interface.
 */
struct pmic_early_priv {
	struct pmic_early_bus *bus;
	struct mutex lock;
	u32 last_val;
	int last_err;
};

static ssize_t spmi_cmd_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct pmic_early_priv *priv = dev_get_drvdata(dev);
	unsigned int sid, reg, val = 0, mask = 0;
	u32 cur = 0;
	int n, ret = 0;

	n = sscanf(buf, "%i %i %i %i", &sid, &reg, &val, &mask);
	if (n < 2)
		return -EINVAL;
	if (sid > 0xf || reg > 0xffff)
		return -EINVAL;

	mutex_lock(&priv->lock);
	if (n == 2) {
		ret = pmic_early_read(priv->bus, sid, reg, &priv->last_val);
	} else {
		if (n == 4) {
			ret = pmic_early_read(priv->bus, sid, reg, &cur);
			if (!ret)
				val = (cur & ~mask) | (val & mask);
		}
		if (!ret)
			ret = pmic_early_write(priv->bus, sid, reg,
					       0xffffffff, val);
	}
	priv->last_err = ret;
	mutex_unlock(&priv->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(spmi_cmd);

static ssize_t spmi_val_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct pmic_early_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%02x %d\n", priv->last_val, priv->last_err);
}
static DEVICE_ATTR_RO(spmi_val);

static struct attribute *pmic_early_attrs[] = {
	&dev_attr_spmi_cmd.attr,
	&dev_attr_spmi_val.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pmic_early);

static int mt6991_pmic_early_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child;
	struct pmic_early_bus *buses;
	struct pmic_early_seq *seqs;
	struct pmic_early_priv *priv;
	int nbuses = 0, nseqs = 0, ret = 0;
	unsigned int count;
	int i;

	count = of_get_child_count(np);
	if (!count)
		return dev_err_probe(dev, -EINVAL, "no child nodes\n");

	buses = devm_kcalloc(dev, count, sizeof(*buses), GFP_KERNEL);
	seqs = devm_kcalloc(dev, count, sizeof(*seqs), GFP_KERNEL);
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!buses || !seqs || !priv)
		return -ENOMEM;
	mutex_init(&priv->lock);

	for_each_available_child_of_node(np, child) {
		if (!of_property_present(child, "reg"))
			continue;
		ret = pmic_early_bus_parse(dev, child, &buses[nbuses]);
		if (ret) {
			of_node_put(child);
			goto done;
		}
		nbuses++;
	}

	for (i = 0; i < nbuses; i++) {
		if (buses[i].spmi) {
			priv->bus = &buses[i];
			break;
		}
	}
	if (priv->bus) {
		dev_set_drvdata(dev, priv);
		ret = devm_device_add_group(dev, &pmic_early_group);
		if (ret)
			goto done;
	}

	for_each_available_child_of_node(np, child) {
		struct pmic_early_seq *seq = &seqs[nseqs];
		struct device_node *busnp;
		char *name;
		int i, r;

		if (!of_property_present(child, "mediatek,steps") &&
		    !of_property_present(child, "mediatek,state"))
			continue;

		name = devm_kstrdup(dev, child->full_name, GFP_KERNEL);
		if (!name) {
			ret = -ENOMEM;
			of_node_put(child);
			goto done;
		}
		seq->name = name;
		if (of_property_present(child, "mediatek,steps")) {
			ret = pmic_early_cells(dev, child, "mediatek,steps",
					       STEP_CELLS, &seq->steps,
					       &seq->nsteps);
			if (ret) {
				of_node_put(child);
				goto done;
			}
		}
		if (of_property_present(child, "mediatek,state")) {
			ret = pmic_early_cells(dev, child, "mediatek,state",
					       STATE_CELLS, &seq->state,
					       &seq->nstate);
			if (ret) {
				of_node_put(child);
				goto done;
			}
		}
		if (of_property_read_u32(child, "mediatek,usid", &seq->usid))
			seq->usid = 0;

		busnp = of_parse_phandle(child, "mediatek,bus", 0);
		if (!busnp) {
			dev_err(dev, "%pOF: missing mediatek,bus\n", child);
			ret = -EINVAL;
			of_node_put(child);
			goto done;
		}
		for (i = 0; i < nbuses; i++)
			if (buses[i].np == busnp)
				break;
		of_node_put(busnp);
		if (i == nbuses) {
			dev_err(dev, "%pOF: unknown mediatek,bus\n", child);
			ret = -EINVAL;
			of_node_put(child);
			goto done;
		}
		seq->bus = &buses[i];
		nseqs++;

		/*
		 * Keep going after a failure: the sequences are independent
		 * and a later one still may be reachable.  Report the first
		 * error at the end.
		 */
		r = pmic_early_run(dev, seq);
		if (r && !ret)
			ret = r;
	}
done:
	if (!nseqs)
		dev_warn(dev, "no sequences to run\n");
	return ret;
}

static const struct of_device_id mt6991_pmic_early_match[] = {
	{ .compatible = "realme,rmx6688-pmic-early" },
	{ /* sentinel */ }
};

static struct platform_driver mt6991_pmic_early_driver = {
	.probe = mt6991_pmic_early_probe,
	.driver = {
		.name = "mt6991-pmic-early",
		.of_match_table = mt6991_pmic_early_match,
	},
};

static int __init mt6991_pmic_early_init(void)
{
	return platform_driver_register(&mt6991_pmic_early_driver);
}
subsys_initcall(mt6991_pmic_early_init);

MODULE_DESCRIPTION("MT6991 early board rail bring-up");
MODULE_LICENSE("GPL v2");
